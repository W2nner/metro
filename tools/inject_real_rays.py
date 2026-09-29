"""First-return ray/box insertion into a real cloud, including occlusion.

The frozen detector re-estimates the corridor from the modified cloud. Labels
are used only here for scoring, never supplied to Detector.process.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from metro_detector.io import read_pcd
from metro_detector.detector import Detector,Config,basis
from metro_detector.rails import extend

def hit_box(directions,lo,hi):
    with np.errstate(divide='ignore',invalid='ignore'):
        a=lo/directions;b=hi/directions
    near=np.minimum(a,b).max(axis=1);far=np.maximum(a,b).min(axis=1)
    return np.where((near>0)&(far>=near),near,np.inf)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='reports/real-ray-injection.json');args=parser.parse_args()
    c=Config(show_nearby=False);R=basis(c);rows=[]
    for name,frame in [('new_data',0),('new_data',5000),('roundT_doubleT',0)]:
        raw,extra=read_pcd(f'D:/lidar/lidar-review/pcd/{name}/{frame:06d}.pcd');p=raw@R.T
        lengths=np.linalg.norm(p,axis=1);valid=lengths>0;directions=p[valid]/lengths[valid,None];base=Detector(c).process(raw,extras=extra)
        rail=base['rail_model']
        if rail is None:continue
        for distance in (15,30,100,150,200,300):
            y=float(extend(rail['center_coefficients'],np.array([distance]),rail['fit_max_m'])[0]);z=float(extend(rail['height_coefficients'],np.array([distance]),rail['fit_max_m'])[0])
            if base.get('corridor_path'):
                path=base['corridor_path'];y=float(np.interp(distance,path['range_m'],path['lateral_m']));z=float(np.interp(distance,path['range_m'],path['floor_m']))
            for label,size in [('minimum_box',[.3,.3,.1]),('person_proxy',[.4,.5,1.7]),('large_box',[1.,1.,1.])]:
                lo=np.array([distance,y-size[1]/2,z]);hi=lo+size
                hit=hit_box(directions,lo,hi);visible=(hit<lengths[valid])&np.isfinite(hit)
                cloud=p[valid].copy();cloud[visible]=directions[visible]*hit[visible,None]
                out=Detector(c).process((cloud@R).astype(np.float32),extras={k:v[valid] for k,v in extra.items()})
                matches=lambda o: bool(np.all(np.asarray(o['center'])>=lo-.25)&np.all(np.asarray(o['center'])<=hi+.25))
                rows.append({'dataset':name,'frame':frame,'forward_m':distance,'object':label,'unique_visible_returns':len(np.unique(cloud[visible],axis=0)),
                    'detected':any(o['supported'] and matches(o) for o in out['objects']),'candidate':any(matches(o) for o in out['objects'])})
    output={'description':'Ray-box insertion into 3 real scans, first-return occlusion, no reflectance/dropout/noise model. Object placement uses baseline estimated rail geometry, not verified survey coordinates. This is a regression experiment, not real-world range validation.','cases':rows}
    Path(args.output).write_text(json.dumps(output,indent=2))
    for d in (15,30,100,150,200,300):
        r=[x for x in rows if x['forward_m']==d];print(d,'cases',len(r),'nonzero_returns',sum(x['unique_visible_returns']>0 for x in r),'detected',sum(x['detected'] for x in r),'candidate',sum(x['candidate'] for x in r),flush=True)

if __name__=='__main__':main()
