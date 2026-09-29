"""Exercise real HTTP paths against an isolated server, never the user's history."""
import unittest,tempfile,subprocess,sys,socket,time,json,urllib.request,urllib.error
from pathlib import Path

class HttpTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.temp=tempfile.TemporaryDirectory(prefix='metro-http-test-');cls.root=Path(cls.temp.name)
  cls.cloud='x,y,z\n'+''.join(f'{y},-10,{z}\n' for z in [-1.35,-1.3,-1.25] for y in [-.15,0,.15])
  (cls.root/'scan.csv').write_text(cls.cloud)
  with socket.socket() as sock:sock.bind(('127.0.0.1',0));cls.port=sock.getsockname()[1]
  cls.base=f'http://127.0.0.1:{cls.port}'
  cls.process=subprocess.Popen([sys.executable,'-m','metro_detector','serve','--data',str(cls.root),'--state',str(cls.root/'state'),'--port',str(cls.port)],cwd=Path(__file__).resolve().parents[1],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
  deadline=time.monotonic()+10
  while time.monotonic()<deadline:
   try:cls.call('/api/health');break
   except Exception:
    if cls.process.poll() is not None:raise RuntimeError(cls.process.stderr.read().decode())
    time.sleep(.05)
  else:raise RuntimeError('Test server failed to start')
 @classmethod
 def tearDownClass(cls):
  cls.process.terminate()
  try:cls.process.wait(timeout=5)
  except subprocess.TimeoutExpired:cls.process.kill();cls.process.wait()
  cls.process.stderr.close();cls.temp.cleanup()
 @classmethod
 def call(cls,path,data=None,raw=False,headers=None):
  body=data if raw else (json.dumps(data).encode() if data is not None else None)
  req=urllib.request.Request(cls.base+path,data=body,headers=headers or {})
  with urllib.request.urlopen(req,timeout=10) as response:
   value=response.read();return json.loads(value) if 'application/json' in response.headers.get('Content-Type','') else value
 def test_01_upload_detection_history_and_review(self):
  self.call('/api/config',{'ground_mode':'fixed','ground_z_m':-1.4})
  upload=self.call('/api/upload?name=uploaded.csv',self.cloud.encode(),True)
  path=f"/api/frame?bag={upload['dataset']}&frame=0"
  result=self.call(path);self.assertTrue(result['obstacle']);self.call(path)
  history=self.call('/api/history');self.assertEqual(history['totals']['total'],1)
  ident=history['rows'][0]['id'];self.call('/api/history/review',{'id':ident,'verdict':'true','note':'test'})
  self.assertEqual(self.call('/api/history/item?id='+str(ident))['verdict'],'true')
  self.assertEqual(self.call('/api/history/item?id='+str(ident))['cloud_kind'],'source')
  self.assertEqual(len(self.call('/api/history/cloud?id='+str(ident))),9*12)
  self.assertEqual(self.call('/api/history')['totals']['total'],1)
  events=self.call('/api/map/events');self.assertEqual(events['positions'],'illustrative')
  self.assertEqual(events['events'][0]['id'],ident)
  self.call('/api/history/review',{'id':ident,'verdict':'false','note':'outside train path'})
  self.assertEqual(self.call('/api/map/events')['events'],[])
  self.assertEqual(self.call('/api/history?verdict=false')['totals']['total'],1)
  self.call('/api/history/review',{'id':ident,'verdict':'true','note':'test'})
  with self.assertRaises(urllib.error.HTTPError):self.call('/api/history/cloud?id=-1')
  self.assertIn(b'uploaded.csv',self.call('/api/history/csv'))
 def test_02_map_and_bad_paths(self):
  result=self.call('/api/map?name=route.json',b'{"points":[[0,0],[10,3]]}',True);self.assertEqual(result['kind'],'route')
  with self.assertRaises(urllib.error.HTTPError) as caught:self.call('/api/source',{'path':str(self.root.parent)})
  self.assertEqual(caught.exception.code,400)
  with self.assertRaises(urllib.error.HTTPError) as caught:self.call('/api/config',{},headers={'Origin':'http://unrelated.test'})
  self.assertEqual(caught.exception.code,403)
 def test_03_http_live_snapshot_and_stop(self):
  self.call('/api/live/upload?name=frame.csv&timestamp_ns=1000000000',self.cloud.encode(),True)
  deadline=time.monotonic()+3
  while time.monotonic()<deadline:
   packet=self.call('/api/live/frame')
   if not packet.get('waiting'):break
   time.sleep(.02)
  self.assertIn('cloud_base64',packet);self.assertEqual(packet['result']['timestamp_ns'],1000000000)
  self.assertTrue(self.call('/api/live/frame?after='+str(packet['sequence']))['unchanged'])
  deadline=time.monotonic()+3
  while time.monotonic()<deadline:
   rows=self.call('/api/history?source=http')['rows']
   if rows and self.call('/api/history/item?id='+str(rows[0]['id']))['cloud_available']:break
   time.sleep(.02)
  self.assertTrue(rows);self.assertEqual(len(self.call('/api/history/cloud?id='+str(rows[0]['id']))),9*12)
  self.call('/api/live/stop',{});self.assertTrue(self.call('/api/live/frame')['waiting'])

 def test_04_uploaded_cloud_survives_restart(self):
  cls=type(self);arguments=cls.process.args
  cls.process.terminate();cls.process.wait(timeout=5);cls.process.stderr.close()
  cls.process=subprocess.Popen(arguments,cwd=Path(__file__).resolve().parents[1],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
  deadline=time.monotonic()+10
  while time.monotonic()<deadline:
   try:catalog=self.call('/api/catalog');break
   except Exception:time.sleep(.05)
  else:self.fail('Server did not restart')
  uploaded=next(v for v in catalog['datasets'] if v['name']=='uploaded.csv')
  result=self.call('/api/frame?bag='+str(uploaded['id'])+'&frame=0')
  self.assertEqual(result['input_points'],9)
  self.assertEqual(self.call('/api/history?verdict=true')['totals']['total'],1)
  ident=self.call('/api/history?source=http')['rows'][0]['id']
  self.assertEqual(self.call('/api/history/item?id='+str(ident))['cloud_kind'],'snapshot')
  self.assertEqual(len(self.call('/api/history/cloud?id='+str(ident))),9*12)

if __name__=='__main__':unittest.main()
