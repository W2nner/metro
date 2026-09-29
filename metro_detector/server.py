"""Offline review, upload and bounded live input. Bind to localhost by default."""
import csv
import json
import threading
import tempfile
import uuid
from dataclasses import asdict
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse,parse_qs
import numpy as np
from .detector import Config,Detector
from .sources import read_cloud,CLOUD_SUFFIXES,bag_info,is_cloud_file
from .streaming import LiveProcessor
from .maps import read_map
from .history import History
from . import __version__
from .diagnostics import is_bag

def serve(root,host,port,config,state_dir=None):
    root=Path(root).resolve() if root else None;web=Path(__file__).resolve().parents[1]/'web'
    state_root=Path(state_dir or 'state').resolve()
    history=History(state_root/'detections.sqlite3')
    saved_uploads=state_root/'uploads';saved_uploads.mkdir(exist_ok=True)
    previews=state_root/'previews';previews.mkdir(exist_ok=True)
    def record_live(result,meta):
        ident=history.record(result,meta.get('source_topic','stream'),meta.get('session_id','stream')+str(meta.get('source_topic','')),meta.get('frame_index'),meta.get('detector_config',asdict(live.detector.config)))
        if ident is not None:
            with live.condition:latest=live.latest
            if latest is not None and latest[2] is result:
                pending=previews/f'{ident}.tmp';pending.write_bytes(display(latest[1]));pending.replace(previews/f'{ident}.bin')
    catalog=[];detector=Detector(config);live=LiveProcessor(config,on_result=record_live);lock=threading.RLock();cache={};sequence=[None,None]
    uploads=tempfile.TemporaryDirectory(prefix='metro-observatory-');upload_root=Path(uploads.name)
    def add_source(path):
        path=Path(path)
        files=[path] if path.is_file() else sorted(p for p in path.iterdir() if is_cloud_file(p))
        if not files:raise ValueError('No PCD/PLY/XYZ/CSV clouds found in this folder')
        stamps=[]
        if path.is_dir() and (path/'frames.csv').exists():
            with (path/'frames.csv').open(encoding='utf-8') as f:mapping={r['pcd_filename']:int(r['timestamp']) for r in csv.DictReader(f)}
            stamps=[mapping.get(p.name,i*100000000) for i,p in enumerate(files)]
        read_cloud(files[0])
        with lock:
            catalog.append(dict(name=path.name,paths=files,stamps=stamps or [i*100000000 for i in range(len(files))]));return len(catalog)-1
    if root:
        if not root.exists():raise ValueError('Data path does not exist: '+str(root))
        if root.is_file():
            if not is_bag(root):add_source(root)
        elif any(is_cloud_file(p) for p in root.iterdir()):add_source(root)
        else:
            import os
            for directory,children,names in os.walk(root):
                children[:]=sorted(v for v in children if v not in ('.git','state','work','__pycache__','node_modules','.venv','cvat','cvat-data'))
                if any(is_cloud_file(Path(directory)/name) for name in names):
                    try:add_source(Path(directory));children[:]=[]
                    except (ValueError,KeyError,UnicodeError):pass
    for path in sorted(saved_uploads.glob('*/*')):
        if is_cloud_file(path):
            try:add_source(path)
            except (ValueError,KeyError,UnicodeError):pass
    def display(xyz):
        valid=xyz[np.isfinite(xyz).all(axis=1)&np.any(xyz!=0,axis=1)]
        return valid[::max(1,int(np.ceil(len(valid)/150000)))].astype('<f4').tobytes()
    def allowed_path(value):
        path=Path(value).expanduser().resolve()
        roots=[upload_root,saved_uploads]+([root if root.is_dir() else root.parent] if root else [])
        if not any(path==r or path.is_relative_to(r) for r in roots):raise ValueError('Path must be inside the --data folder mounted on the server')
        if not path.exists():raise ValueError('Path does not exist on the server')
        return path
    def history_item(ident):
        item=history.get(ident)
        with lock:
            item['dataset_id']=next((n for n,v in enumerate(catalog) if str(v['paths'][0].parent)==item['source_key'] and item['frame'] is not None and 0<=item['frame']<len(v['paths']) and v['paths'][item['frame']].is_file()),None)
        item['cloud_kind']='source' if item['dataset_id'] is not None else ('snapshot' if (previews/f'{ident}.bin').is_file() else 'missing')
        item['cloud_available']=item['cloud_kind']!='missing'
        return item
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,data,kind='application/json',status=200):
            if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False,allow_nan=False).encode('utf-8')
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            try:
                url=urlparse(self.path);q=parse_qs(url.query)
                if url.path=='/api/catalog':
                    with lock:payload={'datasets':[{'id':i,'name':b['name'],'frames':len(b['paths'])} for i,b in enumerate(catalog)],'config':asdict(detector.config),'data_root':str(root) if root else None,'initial_bag':str(root) if root and is_bag(root) else None}
                    return self.send(payload)
                if url.path=='/api/history':return self.send(history.query(q.get('source',[''])[0],q.get('verdict',[''])[0],q.get('confirmed',['0'])[0]=='1',int(q.get('limit',['100'])[0]),int(q.get('offset',['0'])[0])))
                if url.path=='/api/history/item':
                    return self.send(history_item(int(q['id'][0])))
                if url.path=='/api/history/cloud':
                    item=history_item(int(q['id'][0]))
                    if item['cloud_kind']=='source':
                        with lock:path=catalog[item['dataset_id']]['paths'][item['frame']]
                        payload=display(read_cloud(path)[0])
                    elif item['cloud_kind']=='snapshot':payload=(previews/f"{item['id']}.bin").read_bytes()
                    else:return self.send({'error':'Cloud not retained for this historical event'},status=404)
                    return self.send(payload,'application/octet-stream')
                if url.path=='/api/map/events':
                    # Representative saved alarms on an illustrative route, never localization.
                    groups={}
                    for row in history.query(limit=1000)['rows']:
                        if row['verdict']=='false':continue
                        item=history_item(row['id'])
                        if item['cloud_available']:groups.setdefault(item['source_key'],[]).append(row)
                    events=[]
                    for rows in list(groups.values())[:6]:
                        rows.sort(key=lambda r:(r['frame'] if r['frame'] is not None else r['id']))
                        event=rows[len(rows)//2];event['group_count']=len(rows);events.append(event)
                    return self.send({'events':events,'positions':'illustrative'})
                if url.path=='/api/history/csv':
                    import io
                    data=history.query(q.get('source',[''])[0],q.get('verdict',[''])[0],q.get('confirmed',['0'])[0]=='1',1000,int(q.get('offset',['0'])[0]))
                    buffer=io.StringIO();fields=['id','source','frame','timestamp_ns','distance','mode','confirmed','verdict','note'];writer=csv.DictWriter(buffer,fieldnames=fields,extrasaction='ignore');writer.writeheader()
                    export_rows=list(data['rows'])
                    for offset in range(1000,data['totals']['total'],1000):export_rows.extend(history.query(q.get('source',[''])[0],q.get('verdict',[''])[0],q.get('confirmed',['0'])[0]=='1',1000,offset)['rows'])
                    for row in export_rows:
                        safe={k:(chr(39)+str(v) if isinstance(v,str) and v.startswith(('=','+','-','@')) else v) for k,v in row.items()};writer.writerow(safe)
                    return self.send(('\ufeff'+buffer.getvalue()).encode('utf-8'),'text/csv; charset=utf-8')
                if url.path=='/api/health':return self.send({'ok':True,'version':__version__,'stream':live.status()})
                if url.path=='/api/live/status':return self.send(live.status())
                if url.path=='/api/live/frame':
                    with live.condition:latest=live.latest
                    if latest is None:return self.send({'waiting':True,'stream':live.status()})
                    number,xyz,result=latest
                    if q.get('after',[''])[0]==str(number):return self.send({'unchanged':True,'sequence':number,'stream':live.status()})
                    import base64
                    return self.send({'sequence':number,'result':result,'cloud_base64':base64.b64encode(display(xyz)).decode('ascii'),'stream':live.status()})
                if url.path in ('/api/frame','/api/cloud'):
                    b=int(q.get('bag',['0'])[0]);i=int(q.get('frame',['0'])[0])
                    with lock:
                        if b<0 or b>=len(catalog) or i<0 or i>=len(catalog[b]['paths']):raise ValueError('Invalid frame')
                        key=(b,i)
                        if cache.get('key')!=key:
                            xyz,extra=read_cloud(catalog[b]['paths'][i])
                            if sequence!=[b,i-1]:detector.reset()
                            result=detector.process(xyz,catalog[b]['stamps'][i],extra);sequence[:]=[b,i]
                            result.update(frame=i,dataset=catalog[b]['name']);history.record(result,catalog[b]['name'],str(catalog[b]['paths'][0].parent),i,asdict(detector.config));cache.clear();cache.update(key=key,result=result,cloud=display(xyz))
                        payload=cache['result'] if url.path=='/api/frame' else cache['cloud']
                    return self.send(payload,'application/json' if url.path=='/api/frame' else 'application/octet-stream')
                relative='index.html' if url.path=='/' else url.path.lstrip('/');path=(web/relative).resolve()
                if not path.is_relative_to(web) or not path.is_file():return self.send({'error':'not found'},status=404)
                kinds={'.html':'text/html; charset=utf-8','.js':'application/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.md':'text/plain; charset=utf-8'}
                return self.send(path.read_bytes(),kinds.get(path.suffix,'application/octet-stream'))
            except (ValueError,KeyError,IndexError) as exc:self.send({'error':str(exc)},status=400)
            except (BrokenPipeError,ConnectionResetError):pass
            except Exception as exc:self.send({'error':str(exc)},status=500)
        def do_POST(self):
            try:
                origin=self.headers.get('Origin')
                if origin and urlparse(origin).netloc!=self.headers.get('Host'):return self.send({'error':'Cross-origin writes rejected'},status=403)
                url=urlparse(self.path);q=parse_qs(url.query);length=int(self.headers.get('Content-Length','0'))
                upload=url.path in ('/api/upload','/api/map','/api/live/upload')
                if not 0<length<=(256*1024*1024 if upload else 65536):raise ValueError('Body missing or too large (uploads: 256 MiB)')
                self.connection.settimeout(90);body=self.rfile.read(length)
                if len(body)!=length:raise ValueError('Incomplete upload')
                if upload:
                    name=Path(q.get('name',['cloud.pcd'])[0].replace('\\','/')).name
                    suffix=Path(name).suffix.lower();supported=CLOUD_SUFFIXES|({'.json','.geojson'} if url.path=='/api/map' else set())
                    if suffix not in supported:raise ValueError('Unsupported file extension')
                    directory=(saved_uploads if url.path=='/api/upload' else upload_root)/uuid.uuid4().hex;directory.mkdir();path=directory/name;path.write_bytes(body)
                    keep=False
                    try:
                        if url.path=='/api/map':return self.send(read_map(path))
                        if url.path=='/api/live/upload':
                            xyz,extra=read_cloud(path)
                            if live.kind not in ('http','stopped'):raise ValueError('Stop the current stream before HTTP input')
                            live.kind='http';live.submit(xyz,extra,{'timestamp_ns':int(q['timestamp_ns'][0]) if 'timestamp_ns' in q else None,'source_topic':'http'})
                            return self.send({'accepted':True,'stream':live.status()})
                        dataset=add_source(path);keep=True
                        return self.send({'dataset':dataset,'name':name})
                    finally:
                        if not keep:path.unlink(missing_ok=True);directory.rmdir()
                values=json.loads(body)
                if url.path=='/api/history/review':history.review(int(values['id']),values['verdict'],values.get('note',''));return self.send({'ok':True})
                if url.path=='/api/config':
                    with lock:
                        updated=Config(**{**asdict(detector.config),**values}).validate();detector.config=updated;detector.reset();cache.clear();sequence[:]=[None,None];live.configure(updated)
                    return self.send(asdict(updated))
                if url.path=='/api/source':return self.send({'dataset':add_source(allowed_path(values['path']))})
                if url.path=='/api/bag/info':return self.send({'topics':bag_info(allowed_path(values['path']))})
                if url.path=='/api/live/start':
                    if values['kind']=='bag':
                        rate=float(values.get('rate',1))
                        if not np.isfinite(rate) or not .1<=rate<=4:raise ValueError('Playback rate must be 0.1..4')
                        live.start_bag(allowed_path(values['path']),values.get('topic') or None,rate)
                    elif values['kind']=='ros':live.start_ros(values['topic'])
                    else:raise ValueError('Input kind must be bag or ros')
                    return self.send(live.status())
                if url.path=='/api/live/stop':live.stop_source();return self.send(live.status())
                return self.send({'error':'not found'},status=404)
            except Exception as exc:self.send({'error':str(exc)},status=400)
    print(f'Metro detector: http://{host}:{port} | {len(catalog)} sources',flush=True)
    server=ThreadingHTTPServer((host,port),Handler)
    try:server.serve_forever()
    finally:server.server_close();live.close();uploads.cleanup()
