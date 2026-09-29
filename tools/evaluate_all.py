"""Full sequence audit. One worker owns the whole chronological sequence."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse,csv,hashlib,json,time,sys,platform
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from metro_detector.detector import Detector,Config
from metro_detector.io import read_pcd
from dataclasses import asdict

def run_bag(arguments):
    folder,output,ground_truth,first,last=arguments;folder=Path(folder);output=Path(output)
    chunk=f'{folder.name}__{first:06d}_{last:06d}'
    paths=sorted(folder.glob('*.pcd'))
    with (folder/'frames.csv').open(encoding='utf-8') as f:mapping={r['pcd_filename']:r for r in csv.DictReader(f)}
    annotations={r['frame_index']:r['obstacles'] for r in ground_truth if r['bag']==folder.name and r['obstacles']}
    d=Detector(Config());latency=[];wall=[];counts={'frames':0,'single_alarm_frames':0,'confirmed_alarm_frames':0,'uncertain_frames':0,'no_detection_frames':0,'single_alarm_frames_ge150':0,'confirmed_alarm_frames_ge150':0,'nearby_frames':0,'annotated_frames':0,'annotation_matches_nearby':0,'annotation_matches_in_corridor':0,'read_errors':0}
    with (output/(chunk+'.jsonl')).open('w',encoding='utf-8') as log:
        for i in range(max(0,first-2),last):
            path=paths[i]
            begin=time.perf_counter()
            try:
                xyz,extra=read_pcd(path);r=d.process(xyz,int(mapping[path.name]['timestamp']),extra)
                if i<first:continue
                supported=[o for o in r['objects'] if o['supported']];confirmed=[o for o in supported if o['confirmed']]
                counts['frames']+=1;counts['single_alarm_frames']+=bool(supported);counts['confirmed_alarm_frames']+=bool(confirmed)
                counts['uncertain_frames']+=r['status']=='UNCERTAIN';counts['no_detection_frames']+=r['status']=='NO_OBSTACLE_DETECTED'
                counts['single_alarm_frames_ge150']+=any(o['distance_m']>=150 for o in supported)
                counts['confirmed_alarm_frames_ge150']+=any(o['distance_m']>=150 for o in confirmed)
                counts['nearby_frames']+=bool(r['nearby_objects'])
                matched_nearby=False;matched_corridor=False
                if i in annotations:
                    counts['annotated_frames']+=1
                    for gt in annotations[i]:
                        # Export uses source XYZ; transformed to detector basis.
                        center=np.asarray([gt['center'][a] for a in 'xyz']);size=np.asarray([gt['dimensions'][a] for a in 'xyz'])
                        lo=center-size/2;hi=center+size/2
                        def matches(obj):
                            return bool(np.all(np.asarray(obj['raw_center'])>=lo-.2)&np.all(np.asarray(obj['raw_center'])<=hi+.2))
                        matched_nearby |= any(matches(o) for o in r['nearby_objects'])
                        matched_corridor |= any(matches(o) for o in supported)
                    counts['annotation_matches_nearby']+=matched_nearby;counts['annotation_matches_in_corridor']+=matched_corridor
                latency.append(r['processing_ms']);wall.append((time.perf_counter()-begin)*1000)
                row={'frame':i,'status':r['status'],'single':r['obstacle'],'confirmed':bool(confirmed),'distance_m':r['distance_m'],
                     'alarms':[{'distance_m':o['distance_m'],'center':o['center'],'points':o['unique_points'],'confirmed':o['confirmed']} for o in supported],
                     'uncertain_candidates':sum(not o['supported'] for o in r['objects']),'nearby_count':len(r['nearby_objects']),
                     'annotation_nearby_match':matched_nearby,'annotation_in_corridor_match':matched_corridor,'processing_ms':r['processing_ms'],
                     'rail_model_available':bool(r['rail_model']),'points_ge150':r['points_ge_150m']}
                log.write(json.dumps(row)+'\n')
            except Exception as exc:
                counts['read_errors']+=1;log.write(json.dumps({'frame':i,'error':str(exc)})+'\n')
            if i%250==0:
                (output/(chunk+'.progress.json')).write_text(json.dumps({**counts,'last_frame':i,'total':last-first}))
    counts.update(total_expected=last-first)
    (output/(chunk+'.summary.json')).write_text(json.dumps(counts,indent=2))
    return folder.name,chunk,counts,latency,wall

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--data',required=True);parser.add_argument('--annotations',required=True);parser.add_argument('--output',required=True);parser.add_argument('--workers',type=int,default=4);args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    annotation=[r for r in json.loads(Path(args.annotations).read_text(encoding='utf-8'))['frames'] if r['obstacles']]
    source={str(p.relative_to(Path('metro_detector'))):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('metro_detector').glob('*.py')}
    detector_names={'detector.py','rails.py','surfaces.py','range_context.py','trajectory.py','tunnel.py','io.py'}
    report={'detector_sha256':{k:v for k,v in source.items() if k in detector_names},'source_sha256':source,'config':asdict(Config()),'workers':args.workers,'timing_note':'Concurrent audit wall-clock timings; not a single-stream benchmark on evaluation hardware.','annotation_matching':'Detection center inside manual axis-aligned box expanded by 0.2m; not 3D IoU/AP.','platform':platform.platform(),'datasets':{}}
    (output/'report.json').write_text(json.dumps(report,indent=2))
    jobs=[];timings={};read_timings={};completed=[]
    for p in sorted(Path(args.data).iterdir()):
        count=len(list(p.glob('*.pcd'))) if p.is_dir() else 0
        for first in range(0,count,500):jobs.append((str(p),str(output),annotation,first,min(first+500,count)))
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_bag,j) for j in jobs]):
            name,chunk,counts,latency,wall=future.result();completed.append(chunk)
            target=report['datasets'].setdefault(name,{k:0 for k in counts})
            for key,value in counts.items():target[key]+=value
            timings.setdefault(name,[]).extend(latency);read_timings.setdefault(name,[]).extend(wall)
            report['completed_chunks']=completed
            (output/'report.json').write_text(json.dumps(report,indent=2));print(chunk,'frames',counts['frames'],'alarms',counts['single_alarm_frames'],flush=True)
    for name,counts in report['datasets'].items():
        latency=timings[name];wall=read_timings[name]
        counts.update(p50_ms=float(np.median(latency)) if latency else None,p95_ms=float(np.quantile(latency,.95)) if latency else None,p99_ms=float(np.quantile(latency,.99)) if latency else None,read_and_process_p95_ms=float(np.quantile(wall,.95)) if wall else None)
        (output/(name+'.summary.json')).write_text(json.dumps(counts,indent=2))
    report['complete']=all(v['frames']==v['total_expected'] and v['read_errors']==0 for v in report['datasets'].values())
    (output/'report.json').write_text(json.dumps(report,indent=2));print('COMPLETE',report['complete'],flush=True)

if __name__=='__main__':main()
