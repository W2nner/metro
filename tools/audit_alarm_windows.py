"""Reprocess every baseline alarm and its neighbours, without history/UI labels."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from metro_detector.detector import Detector
from metro_detector.io import read_pcd

def main():
    parser=argparse.ArgumentParser()
    for key in ('data','baseline','output'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--positive-dataset',required=True,help='User-labelled positive sequence; evaluation only')
    args=parser.parse_args();alarms={}
    for file in Path(args.baseline).glob('*__*.jsonl'):
        name=file.name.split('__')[0]
        for line in file.read_text().splitlines():
            row=json.loads(line)
            if row.get('single'):alarms.setdefault(name,{})[row['frame']]=row
    rows=[];windows=[]
    for name,baseline in alarms.items():
        files=sorted((Path(args.data)/name).glob('*.pcd'));detector=Detector();last=None
        indices=sorted({j for i in baseline for j in range(max(0,i-3),min(len(files),i+4))})
        for index in indices:
            if last is None or index!=last+1:detector.reset()
            xyz,extra=read_pcd(files[index]);r=detector.process(xyz,index*100000000,extra);last=index
            row={'dataset':name,'frame':index,'status':r['status'],'obstacle':r['obstacle'],
                 'confirmed':any(o['supported'] and o['confirmed'] for o in r['objects']),
                 'user_label':'positive_sequence' if name==args.positive_dataset else 'negative_sequence'}
            windows.append(row)
            if index in baseline:
                row={**row,'previous_distance_m':baseline[index]['distance_m'],'old_objects':[]}
                path=r['corridor_path']
                for old in baseline[index]['alarms']:
                    x=old['center'][0]
                    row['old_objects'].append({'center':old['center'],**{key:float(np.interp(x,path['range_m'],path[key])) for key in ('lateral_m','uncertainty_lateral_m')}})
                rows.append(row)
        print(name,len(indices),'window frames checked',flush=True)
    output={'baseline_alarm_frames':len(rows),'window_frames':len(windows),
            'negative_alarm_frames_after':sum(r['obstacle'] and r['user_label']=='negative_sequence' for r in windows),
            'retained_positive_alarm_frames':sum(r['obstacle'] and r['user_label']=='positive_sequence' for r in rows),
            'note':'Labels from user sequence descriptions. Not hidden-test or surveyed trajectory ground truth. UNCERTAIN is not a clear-path decision.',
            'alarms':rows,'windows':windows}
    Path(args.output).write_text(json.dumps(output,indent=2));print({k:v for k,v in output.items() if k not in ('alarms','windows')})

if __name__=='__main__':main()
