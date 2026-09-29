import argparse
import json
import sys
from pathlib import Path
from contextlib import nullcontext
from . import __version__
from .detector import Config, Detector
from .sources import cloud_sequence, bag_frames


def main():
    parser=argparse.ArgumentParser(description='Metro LiDAR obstacle detector')
    parser.add_argument('--version',action='version',version=__version__)
    sub=parser.add_subparsers(dest='command',required=True)
    inspect=sub.add_parser('inspect',help='Inspect a cloud, cloud directory or ROS 2 bag')
    inspect.add_argument('input')
    doctor=sub.add_parser('doctor',help='Check runtime dependencies')
    doctor.add_argument('--require-ros',action='store_true')
    for name in ('process','serve','ros'):
        p=sub.add_parser(name);p.add_argument('--config')
        if name=='process':
            p.add_argument('input');p.add_argument('--bag',action='store_true');p.add_argument('--topic')
            p.add_argument('--output',required=True,help='JSONL file, or - for stdout')
            p.add_argument('--summary',help='Optional summary JSON file')
            p.add_argument('--limit',type=int)
        elif name=='serve':
            p.add_argument('--data');p.add_argument('--host',default='127.0.0.1')
            p.add_argument('--port',type=int,default=8190);p.add_argument('--state',default='state')
        else:p.add_argument('--topic',default='/lidar_points')
    args=parser.parse_args()
    if args.command=='doctor':
        from .diagnostics import doctor
        report=doctor(args.require_ros);print(json.dumps(report,indent=2));return 0 if report['ok'] else 2
    if args.command=='inspect':
        from .diagnostics import inspect_source
        print(json.dumps(inspect_source(args.input),indent=2));return 0
    config=Config(**json.loads(Path(args.config).read_text(encoding='utf-8-sig'))) if args.config else Config()
    config.validate()
    if args.command=='serve':
        from .server import serve
        serve(args.data,args.host,args.port,config,args.state);return 0
    if args.command=='ros':
        from .ros_node import run
        run(args.topic,config);return 0
    if args.limit is not None and args.limit<1:raise ValueError('--limit must be positive')
    from .diagnostics import is_bag
    detector=Detector(config)
    stream=bag_frames(args.input,args.topic) if args.bag or is_bag(args.input) else cloud_sequence(args.input)
    destination=Path(args.output) if args.output!='-' else None
    if destination:
        if destination.resolve()==Path(args.input).resolve():raise ValueError('Output must differ from input')
        destination.parent.mkdir(parents=True,exist_ok=True)
    if args.summary:
        summary_path=Path(args.summary).resolve()
        if summary_path==Path(args.input).resolve() or (destination and summary_path==destination.resolve()):
            raise ValueError('Summary must differ from input and output')
    stats={'version':__version__,'frames':0,'alarm_frames':0,'uncertain_frames':0,'processing_total_ms':0.,'max_processing_ms':0.,'nearest_obstacle_m':None}
    from itertools import islice
    if args.limit is not None:stream=islice(stream,args.limit)
    with (destination.open('w',encoding='utf-8') if destination else nullcontext(sys.stdout)) as output:
        for xyz,extra,meta in stream:
            result=detector.process(xyz,meta.get('timestamp_ns'),extra);result['source']=meta
            output.write(json.dumps(result,allow_nan=False)+'\n')
            stats['frames']+=1;stats['alarm_frames']+=int(result['obstacle'])
            stats['uncertain_frames']+=int(result['status']=='UNCERTAIN')
            stats['processing_total_ms']+=result['processing_ms']
            stats['max_processing_ms']=max(stats['max_processing_ms'],result['processing_ms'])
            if result['distance_m'] is not None:stats['nearest_obstacle_m']=min(stats['nearest_obstacle_m'] or float('inf'),result['distance_m'])
            if stats['frames']%100==0:print('Processed',stats['frames'],'frames',file=sys.stderr,flush=True)
    stats['mean_processing_ms']=stats['processing_total_ms']/max(1,stats['frames'])
    if args.summary:
        target=Path(args.summary);target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(stats,indent=2),encoding='utf-8')
    print(json.dumps(stats),file=sys.stderr)
    return 0


if __name__=='__main__':
    try:sys.exit(main())
    except KeyboardInterrupt:sys.exit(130)
    except Exception as exc:
        print('ERROR: '+str(exc),file=sys.stderr);sys.exit(2)
