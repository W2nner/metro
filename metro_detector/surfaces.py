"""Reject a candidate only when an external surface continues through it."""
import numpy as np
from scipy.spatial import cKDTree

def foreground_edges(points,objects,min_near,min_far,far_threshold,required_after=None,corridor_width=2.1,corridor_height=3.):
    candidates=[o for o in objects if (not o['angular_support'] or (required_after is not None and o['center'][0]>required_after)) and o['geometry_support'] and
                o['unique_points']>=(min_far if o['distance_m']>=far_threshold else min_near)]
    if not candidates:return
    az=np.arctan2(points[:,1],points[:,0]);el=np.arctan2(points[:,2],np.hypot(points[:,0],points[:,1]))
    angular=np.column_stack([az,el*5]);tree=cKDTree(angular,balanced_tree=False,compact_nodes=False)
    ranges=np.linalg.norm(points,axis=1)
    for obj in candidates:
        lo=np.asarray(obj['min']);hi=np.asarray(obj['max'])
        ids=np.flatnonzero(np.all((points>=lo-1e-5)&(points<=hi+1e-5),axis=1))
        if len(ids)<2:continue
        ordered=ids[np.argsort(el[ids])]
        levels=np.unique(el[ordered[np.linspace(0,len(ordered)-1,min(12,len(ordered))).astype(int)]])
        rows=[]
        for e in levels:
            row_ids=ids[np.abs(el[ids]-e)<.00015]
            a0,a1=az[row_ids].min(),az[row_ids].max()
            farthest=ranges[row_ids].max()
            good_sides=0
            for sign,target in ((-1,a0-.00175),(1,a1+.00175)):
                _,nearest=tree.query([target,e*5],k=min(64,len(points)))
                nearest=np.atleast_1d(nearest)
                side=(az[nearest]<a0-.0002) if sign<0 else (az[nearest]>a1+.0002)
                valid=side&(np.abs(el[nearest]-e)<.00015)&(np.abs(az[nearest]-target)<.002)
                if valid.any():
                    chosen=nearest[valid][np.argmin(np.abs(az[nearest[valid]]-target))]
                    if ranges[chosen]>farthest+max(.3,obj['distance_m']*.008):good_sides+=1
            rows.append(good_sides)
        obj['depth_edge_rows']=rows
        visible_levels=levels[np.asarray(rows)>0]
        both_levels=levels[np.asarray(rows)==2]
        paired=len(both_levels)>=2 and np.ptp(both_levels)>.0003
        # A neighbouring rail/wall may hide one silhouette edge. Several
        # independent laser rows can still establish a foreground object.
        size=obj['dimensions']
        compact_front=size[0]<=max(size[1],size[2])
        partial=compact_front and obj['angular_support'] and obj['unique_points']>=12 and len(visible_levels)>=2 and np.ptp(visible_levels)>.0003
        if paired or partial:
            obj['supported']=True;obj['evidence']='foreground_depth_edges' if paired else 'foreground_partial_occlusion';obj['foreground_edges']=True
        elif required_after is not None and obj['center'][0]>required_after:
            size=obj['dimensions']
            blocking_face=obj['supported'] and obj['unique_points']>=12 and size[0]<1. and size[1]>=corridor_width*.8 and size[2]>=corridor_height*.5
            obj['supported']=blocking_face;obj['evidence']='blocking_face' if blocking_face else 'depth_contrast_unconfirmed'

def continuing_surfaces(points,objects):
    supported=[o for o in objects if o['supported']]
    if not supported:return
    near=points[:,0]<35
    sample=np.concatenate([points[near][::3],points[~near]])
    tree=cKDTree(sample,balanced_tree=False,compact_nodes=False)
    rng=np.random.default_rng(912)
    for obj in supported:
        center=np.asarray(obj['center']);lo=np.asarray(obj['min']);hi=np.asarray(obj['max'])
        radius=max(1.2,min(3.,obj['distance_m']*.018))
        ids=tree.query_ball_point(center,radius)
        context=sample[ids]
        external=np.any((context<lo-.08)|(context>hi+.08),axis=1)
        context=context[external]
        if len(context)<16:continue
        context=np.unique(context,axis=0)
        if len(context)<12:continue
        context=context[::max(1,len(context)//400)]
        picks=rng.integers(0,len(context),(96,3))
        a=context[picks[:,0]];b=context[picks[:,1]];c=context[picks[:,2]]
        normals=np.cross(b-a,c-a);length=np.linalg.norm(normals,axis=1)
        good=length>.2;normals=normals[good]/length[good,None];a=a[good]
        if not len(a):continue
        offsets=np.einsum('ij,ij->i',normals,a)
        residual=np.abs(context@normals.T-offsets)
        tolerance=.035+min(.025,obj['distance_m']*.000125)
        mask=residual<tolerance
        cluster=points[np.all((points>=lo-1e-5)&(points<=hi+1e-5),axis=1)]
        if not len(cluster):continue
        agrees=np.max(np.abs(cluster@normals.T-offsets),axis=0)<tolerance
        mask[:,~agrees]=False
        best=int(np.argmax(mask.sum(axis=0)));inliers=context[mask[:,best]]
        if len(inliers)<12 or len(inliers)<len(context)*.25:continue
        _,singular,vectors=np.linalg.svd(inliers-inliers.mean(axis=0),full_matrices=False)
        projected=(inliers-inliers.mean(axis=0))@vectors.T
        spans=np.ptp(projected,axis=0)
        if spans[0]<1.2 or spans[1]<.6:continue
        normal=vectors[-1];origin=inliers.mean(axis=0)
        # Evaluate actual cluster points, not its box corners (a tilted plane
        # has non-planar AABB corners).
        cluster=points[np.all((points>=lo-1e-5)&(points<=hi+1e-5),axis=1)]
        if len(cluster) and np.max(np.abs((cluster-origin)@normal))<tolerance:
            obj['supported']=False;obj['evidence']='continuing_surface';obj['background']=True
