from .range_context import check_context
"""Geometry-only detector: no bag names, annotations, maps or learned backgrounds.

Internal axes: X forward, Y left, Z up. Outputs are in this explicitly reported
coordinate basis. Distances are Euclidean distances from the LiDAR origin.
"""
from dataclasses import dataclass, field, asdict
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from .tunnel import profile,interpolate,centerline
from .rails import track_model,extend,uncertainty
from .surfaces import continuing_surfaces,foreground_edges
from .trajectory import build_path,sample_path

@dataclass
class Config:
    forward_axis:str='-y'
    up_axis:str='+z'
    yaw_deg:float=0.0
    lateral_offset_m:float=0.0
    width_m:float=2.1
    height_m:float=3.0
    margin_enabled:bool=False
    margin_lateral_m:float=0.2
    margin_top_m:float=0.2
    min_range_m:float=1.5
    max_range_m:float=320.0
    ground_mode:str='auto'
    ground_z_m:float=-1.4
    ground_clearance_m:float=0.025
    floor_fit_max_m:float=40.0
    calibration_verified:bool=False
    self_masks:list=field(default_factory=list)
    min_points_near:int=6
    min_points_far:int=2
    far_threshold_m:float=100.0
    voxel_near_m:float=0.16
    voxel_far_m:float=0.45
    mode:str='single'
    confirmation_hits:int=2
    confirmation_window:int=3
    max_speed_mps:float=23.62
    temporal_max_gap_s:float=0.5
    rail_filter_enabled:bool=True
    corridor_model:str='tunnel_relative'
    show_nearby:bool=True

    def validate(self):
        axis_vector(self.forward_axis);axis_vector(self.up_axis)
        if self.forward_axis[-1]==self.up_axis[-1]:raise ValueError('Forward and up axes must differ')
        if self.mode not in ('single','confirmed'):raise ValueError('mode: single or confirmed')
        if self.ground_mode not in ('auto','fixed'):raise ValueError('ground_mode: auto or fixed')
        if self.corridor_model not in ('straight','tunnel_relative'):raise ValueError('Invalid corridor_model')
        for k in ('confirmation_hits','confirmation_window','min_points_near','min_points_far'):
            if type(getattr(self,k)) is not int:raise ValueError(k+' must be an integer')
        for k in ('margin_enabled','calibration_verified','rail_filter_enabled','show_nearby'):
            if type(getattr(self,k)) is not bool:raise ValueError(k+' must be boolean')
        for k in ('width_m','height_m','max_range_m','voxel_near_m','voxel_far_m','floor_fit_max_m','max_speed_mps','temporal_max_gap_s','far_threshold_m'):
            v=getattr(self,k)
            if not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0:raise ValueError(k+' must be positive and finite')
        for k in ('yaw_deg','lateral_offset_m','ground_z_m'):
            if not math.isfinite(getattr(self,k)):raise ValueError(k+' must be finite')
        for k in ('margin_lateral_m','margin_top_m','min_range_m','ground_clearance_m'):
            if not math.isfinite(getattr(self,k)) or getattr(self,k)<0:raise ValueError(k+' must be nonnegative')
        if not 1<=self.confirmation_hits<=self.confirmation_window<=10:raise ValueError('Invalid confirmation window')
        if self.max_range_m<=self.min_range_m or self.max_range_m>2000:raise ValueError('Invalid range')
        if self.min_points_near<1 or self.min_points_far<1:raise ValueError('Point counts must be >= 1')
        for mask in self.self_masks:
            lo=np.asarray(mask['min'],float);hi=np.asarray(mask['max'],float)
            if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or not (hi>lo).all():raise ValueError('Invalid self mask')
        return self

def axis_vector(name):
    if name not in ('+x','-x','+y','-y','+z','-z'):raise ValueError('Invalid axis '+str(name))
    v=np.zeros(3,dtype=np.float32);v['xyz'.index(name[1])]=1 if name[0]=='+' else -1
    return v

def basis(config):
    f=axis_vector(config.forward_axis);u=axis_vector(config.up_axis);l=np.cross(u,f)
    angle=math.radians(config.yaw_deg);c,s=math.cos(angle),math.sin(angle)
    return np.stack([c*f+s*l,-s*f+c*l,u])

def estimate_floor(points,config):
    if config.ground_mode=='fixed':return np.array([0.,0.,config.ground_z_m]),{'valid':True,'method':'configured','support':0}
    f,l,z=points.T
    mask=(f>2)&(f<config.floor_fit_max_m)&(np.abs(l-config.lateral_offset_m)<1.6)&(z<-.45)&(z>-5)
    samples=points[mask]
    if len(samples)<100:return np.array([0.,0.,config.ground_z_m]),{'valid':False,'method':'fallback','support':len(samples)}
    # The densest lower surface, rather than the lowest drain/trench returns.
    anchors=[]
    for start in np.arange(2,config.floor_fit_max_m,3):
        s=samples[(samples[:,0]>=start)&(samples[:,0]<start+3)]
        if len(s)<30:continue
        histogram,edges=np.histogram(s[:,2],bins=np.arange(-5,-.39,.04))
        level=edges[np.argmax(histogram)]+.02
        patch=s[np.abs(s[:,2]-level)<.09]
        if len(patch)>15:anchors.append(patch[::max(1,len(patch)//250)])
    if len(anchors)<3:return np.array([0.,0.,config.ground_z_m]),{'valid':False,'method':'fallback','support':len(samples)}
    a=np.concatenate(anchors);A=np.column_stack([a[:,0],a[:,1],np.ones(len(a))])
    plane=np.linalg.lstsq(A,a[:,2],rcond=None)[0]
    for _ in range(3):
        residual=np.abs(a[:,2]-A@plane);keep=residual<max(.08,min(.18,float(np.median(residual)*2.5)))
        if keep.sum()<40:break
        plane=np.linalg.lstsq(A[keep],a[keep,2],rcond=None)[0]
    valid=abs(plane[0])<.1 and abs(plane[1])<.2 and -5<plane[2]<-.3
    if not valid:return np.array([0.,0.,config.ground_z_m]),{'valid':False,'method':'fallback','support':len(a)}
    return plane,{'valid':True,'method':'robust_single_cloud_plane','support':len(a),
                  'residual_m':float(np.median(np.abs(a[:,2]-A@plane))),
                  'extrapolated_beyond_m':config.floor_fit_max_m}

def connected_cells(points,cell_size):
    """26-connected occupied cells; full-resolution coordinates retained for boxes."""
    if not len(points):return []
    cells=np.floor(points/cell_size).astype(np.int32)
    # Integer voxel keys preserve lexicographic XYZ ordering without sorting
    # structured 3-column records. Fall back if the packing could overflow.
    shifted=cells.astype(np.int64)-cells.min(axis=0).astype(np.int64)
    spans=shifted.max(axis=0)+1
    if int(spans[0])*int(spans[1])*int(spans[2])<np.iinfo(np.int64).max:
        keys=(shifted[:,0]*spans[1]+shifted[:,1])*spans[2]+shifted[:,2]
        _,first,inverse=np.unique(keys,return_index=True,return_inverse=True)
        unique=cells[first]
    else:unique,inverse=np.unique(cells,axis=0,return_inverse=True)
    # Chebyshev radius 1 is exactly the same 26-cell neighbourhood as the
    # reference Python union-find; SciPy performs graph construction in C.
    pairs=cKDTree(unique).query_pairs(1.,p=np.inf,output_type='ndarray')
    graph=coo_matrix((np.ones(len(pairs),dtype=np.uint8),(pairs[:,0],pairs[:,1])),shape=(len(unique),len(unique))).tocsr()
    _,labels=connected_components(graph,directed=False)
    roots=labels[inverse]
    order=np.argsort(roots,kind='stable');split=np.flatnonzero(np.diff(roots[order]))+1
    return np.split(order,split)

class Detector:
    def __init__(self,config=None):
        self.config=(config or Config()).validate();self.history=[];self.last_stamp=None
    def reset(self):self.history=[];self.last_stamp=None
    def process(self,xyz,timestamp_ns=None,extras=None):
        start=time.perf_counter();c=self.config;R=basis(c)
        raw=np.asarray(xyz,dtype=np.float32)
        if raw.ndim!=2 or raw.shape[1]!=3:raise ValueError('Expected Nx3 XYZ array')
        valid=np.isfinite(raw).all(axis=1)&np.any(raw!=0,axis=1)
        ring=None
        if extras is not None and 'ring' in extras:
            value=np.asarray(extras['ring'])
            if value.shape!=(len(raw),):raise ValueError('ring must have one scalar per XYZ point')
            if np.isfinite(value).all() and len(np.unique(value))<=256:ring=value[valid]
        raw=raw[valid]
        # Axis-wise expression avoids an unnecessary BLAS thread pool for Nx3.
        p=np.empty_like(raw)
        for i in range(3):p[:,i]=raw[:,0]*R[i,0]+raw[:,1]*R[i,1]+raw[:,2]*R[i,2]
        distance=np.sqrt(np.einsum('ij,ij->i',p,p))
        forward=(p[:,0]>0)&(distance<=c.max_range_m)&(distance>=c.min_range_m)
        p=p[forward];distance=distance[forward]
        if ring is not None:ring=ring[forward]
        for mask in c.self_masks:
            inside=((p>=np.asarray(mask['min']))&(p<=np.asarray(mask['max']))).all(axis=1)
            p=p[~inside];distance=distance[~inside]
            if ring is not None:ring=ring[~inside]
        plane,floor_info=estimate_floor(p,c)
        floor=p[:,0]*plane[0]+p[:,1]*plane[1]+plane[2]
        sections=[]
        if c.ground_mode=='auto':
            sections=profile(p,c.lateral_offset_m)
            if len(sections)>=3:
                floor,roof,wall_left,wall_right=interpolate(p,sections,c.ground_z_m)
                floor_info['method']='single_cloud_cross_sections'
            else:roof=np.full(len(p),np.inf);wall_left=np.full(len(p),-np.inf);wall_right=np.full(len(p),np.inf)
        else:roof=np.full(len(p),np.inf);wall_left=np.full(len(p),-np.inf);wall_right=np.full(len(p),np.inf)
        height=p[:,2]-floor
        half=c.width_m/2+(c.margin_lateral_m if c.margin_enabled else 0)
        top=c.height_m+(c.margin_top_m if c.margin_enabled else 0)
        centers,trace=centerline(p,sections,c.lateral_offset_m,interpolate_points=c.ground_mode!='auto') if c.corridor_model=='tunnel_relative' else (np.full(len(p),c.lateral_offset_m),None)
        measured_centers=centers.copy();measured_floor=floor.copy()
        background=np.zeros(len(p),bool);alignment=None
        rails=track_model(p) if c.ground_mode=='auto' else None
        if rails:
            if c.corridor_model=='tunnel_relative':centers=extend(rails['center_coefficients'],p[:,0],rails['fit_max_m'])+c.lateral_offset_m
            floor=extend(rails['height_coefficients'],p[:,0],rails['fit_max_m'])+rails['cross_slope']*(p[:,1]-centers)
            floor_info['method']='paired_rail_ridges';floor_info['valid']=True
        path=None
        if c.ground_mode=='auto' and c.corridor_model=='tunnel_relative':
            path=build_path(sections,rails,c.lateral_offset_m,plane,c.max_range_m)
            centers=sample_path(path,p[:,0],'lateral_m')
            floor=sample_path(path,p[:,0],'floor_m')
            if rails:floor+=rails['cross_slope']*(p[:,1]-centers)
        height=p[:,2]-floor
        interior=(p[:,1]>wall_left+.15)&(p[:,1]<wall_right-.15)&((p[:,0]<15)|(p[:,2]<roof-.15))
        roi=(np.abs(p[:,1]-centers)<=half)&(height>c.ground_clearance_m)&(height<top)&interior&~background
        candidates=p[roi];candidate_distance=distance[roi]
        uncertainty_y=uncertainty(rails,candidates[:,0],'y') if rails else np.full(len(candidates),0. if c.ground_mode=='fixed' else .5)
        uncertainty_z=uncertainty(rails,candidates[:,0],'z') if rails else np.full(len(candidates),0. if c.ground_mode=='fixed' else .25)
        if path:
            uncertainty_y=sample_path(path,candidates[:,0],'uncertainty_lateral_m')
            uncertainty_z=sample_path(path,candidates[:,0],'uncertainty_height_m')
        elif rails:
            beyond=candidates[:,0]>rails['fit_max_m']
            uncertainty_y[beyond]+=np.abs(centers[roi][beyond]-measured_centers[roi][beyond])*.5
            uncertainty_z[beyond]+=np.abs(floor[roi][beyond]-measured_floor[roi][beyond])*.5
        stable=(np.abs(candidates[:,1]-centers[roi])<half-uncertainty_y)&(height[roi]>c.ground_clearance_m+uncertainty_z)&(height[roi]<top-uncertainty_z)
        alignment_verified=c.ground_mode=='fixed' or rails is not None
        if not alignment_verified:stable[:]=False
        objects=[];suppressed=0
        # Disjoint distance bands retain singleton evidence at long range.
        for lower,upper,cell in ((0,50,c.voxel_near_m),(50,100,.28),(100,c.max_range_m+1,c.voxel_far_m)):
            sel=(candidate_distance>=lower)&(candidate_distance<upper)
            cloud=candidates[sel];ranges=candidate_distance[sel]
            stable_band=stable[sel]
            for indices in connected_cells(cloud,cell):
                cluster=cloud[indices];ds=ranges[indices]
                lo=cluster.min(axis=0);hi=cluster.max(axis=0);size=hi-lo
                nearest=float(ds.min());count=len(indices)
                # Repeated returns at identical coordinates are not independent
                # evidence. The source may contain duplicate XYZ values.
                unique,unique_index=np.unique(cluster,axis=0,return_index=True)
                evidence_count=len(unique)
                heights=cluster[:,2]-(cluster[:,0]*plane[0]+cluster[:,1]*plane[1]+plane[2])
                # Only low, narrow, longitudinal structures are rail-like.
                # A sloping rail has a wide axis-aligned box. Measure thickness
                # about its fitted longitudinal line instead of box width.
                rail_like=False
                if c.rail_filter_enabled and size[0]>2 and count>=20 and float(heights.max())<.3:
                    line=np.polyfit(cluster[:,0],cluster[:,1],1)
                    residual=cluster[:,1]-np.polyval(line,cluster[:,0])
                    rail_like=float(np.quantile(residual,.95)-np.quantile(residual,.05))<.16
                if rail_like:suppressed+=count;continue
                threshold=c.min_points_far if nearest>=c.far_threshold_m else c.min_points_near
                azimuth=np.arctan2(cluster[:,1],cluster[:,0])
                elevation=np.arctan2(cluster[:,2],np.hypot(cluster[:,0],cluster[:,1]))
                # One scan column or one laser row can be a normal tunnel
                # return. Retain it as ambiguous evidence, rather than claiming
                # a supported 3D obstacle merely because values repeat.
                angular_support=float(np.ptp(azimuth))>1e-4 and float(np.ptp(elevation))>1e-4
                # The minimum return count must also hold inside the geometry
                # confidence band, not just inside its uncertain boundary.
                geometry_support=bool(np.mean(stable_band[indices])>=.5 and
                                      np.count_nonzero(stable_band[indices][unique_index])>=threshold)
                sufficient=evidence_count>=threshold and angular_support and geometry_support
                center=(lo+hi)/2
                objects.append({'id':len(objects),'center':center.tolist(),'min':lo.tolist(),'max':hi.tolist(),
                    'dimensions':size.tolist(),'distance_m':nearest,'points':count,'unique_points':evidence_count,'evidence':'supported' if sufficient else 'sparse',
                    'confidence':round(min(.99,.25+math.log1p(count)/10),3),'supported':sufficient,
                    'angular_support':angular_support,
                    'geometry_support':geometry_support,
                    'raw_center':(center@R).tolist(),'confirmed':False})
        foreground_edges(p,objects,c.min_points_near,c.min_points_far,c.far_threshold_m,
                         rails['fit_max_m'] if rails else (40. if c.ground_mode=='auto' else None),half*2,top)
        continuing_surfaces(p,objects)
        check_context(p,objects,ring,c.min_points_near,c.min_points_far,c.far_threshold_m)
        objects=[o for o in objects if not o.get('background')]
        objects.sort(key=lambda o:o['distance_m'])
        nearby=[]
        if c.show_nearby:
            neighbour=(np.abs(p[:,1]-centers)>half+.05)&(np.abs(p[:,1]-centers)<half+2.)&(height>.12)&(height<2.6)&interior
            outside=p[neighbour]
            for indices in connected_cells(outside,.20):
                cluster=outside[indices]
                if len(cluster)<12:continue
                lo=cluster.min(axis=0);hi=cluster.max(axis=0);size=hi-lo
                if size[0]>3.5 or size[1]>1.5 or size[2]<.35:continue
                center=(lo+hi)/2
                nearby.append({'center':center.tolist(),'min':lo.tolist(),'max':hi.tolist(),'dimensions':size.tolist(),
                    'distance_m':float(np.linalg.norm(cluster,axis=1).min()),'points':len(cluster),'type':'unknown',
                    'relation':'outside_corridor','raw_center':(center@R).tolist()})
            nearby.sort(key=lambda o:o['distance_m'])
        stamp=float(timestamp_ns)/1e9 if timestamp_ns is not None else time.monotonic()
        if self.last_stamp is not None and (stamp<=self.last_stamp or stamp-self.last_stamp>c.temporal_max_gap_s):self.reset()
        for obj in objects:
            hits=1
            for previous_stamp,previous in self.history[-(c.confirmation_window-1):] if c.confirmation_window>1 else []:
                allowed=c.max_speed_mps*(stamp-previous_stamp)+1.
                if any(o['supported'] and abs(o['center'][0]-obj['center'][0])<=allowed and abs(o['center'][1]-obj['center'][1])<.8 and abs(o['center'][2]-obj['center'][2])<1 for o in previous):hits+=1
            obj['confirmed']=hits>=c.confirmation_hits
            obj['confirmation_hits']=hits
        self.history.append((stamp,objects));self.history=self.history[-c.confirmation_window:];self.last_stamp=stamp
        selected=[o for o in objects if o['supported'] and (c.mode=='single' or o['confirmed'])]
        nearest=min((o['distance_m'] for o in selected),default=None)
        warnings=[]
        if not c.calibration_verified:warnings.append('CALIBRATION_UNVERIFIED')
        if not c.self_masks:warnings.append('SELF_MASK_NOT_CONFIGURED')
        if not floor_info['valid']:warnings.append('GROUND_ESTIMATE_UNRELIABLE')
        if not alignment_verified:warnings.append('PATH_ALIGNMENT_UNVERIFIED')
        if not len(p):warnings.append('NO_VALID_FORWARD_POINTS')
        warnings.append('TUNNEL_RELATIVE_CORRIDOR_UNVERIFIED' if trace else 'STRAIGHT_CORRIDOR_ASSUMPTION')
        long_points=int((distance>=150).sum())
        if long_points<10:warnings.append('SPARSE_OR_OCCLUDED_BEYOND_150M')
        status='OBSTACLE' if selected else ('UNCERTAIN' if objects or not alignment_verified or not floor_info['valid'] or not len(p) else 'NO_OBSTACLE_DETECTED')
        return {'schema_version':1,'status':status,'obstacle':bool(selected),'distance_m':nearest,
                'candidate_distance_m':min((o['distance_m'] for o in objects),default=None),
                'objects':objects,'mode':c.mode,'timestamp_ns':timestamp_ns,
                'nearby_objects':nearby,
                'input_points':len(xyz),'valid_points':int(valid.sum()),'roi_points':len(candidates),
                'suppressed_rail_points':suppressed,'points_ge_150m':long_points,
                'max_observed_range_m':float(distance.max()) if len(distance) else 0.,
                'ground_plane':plane.tolist(),'ground':floor_info,'warnings':warnings,
                'tunnel_sections':sections,
                'corridor_trace':trace,
                'corridor_path':path,
                'section_alignment':alignment,
                'rail_model':rails,
                'basis_raw_to_forward_left_up':R.tolist(),'corridor':{'center_lateral_m':c.lateral_offset_m,'half_width_m':half,'height_m':top},
                'processing_ms':(time.perf_counter()-start)*1000}
