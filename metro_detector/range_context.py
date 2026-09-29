"""Full-cloud range-image context for ROI fragments.

Independent implementation of the adjacent-ray angle criterion described by
Bogoslavskyi & Stachniss, IROS 2016 (PRBonn/depth_clustering). No external code
or pretrained weights are bundled. Ring IDs are optional; XYZ uses a 0.1-degree
projection. This is evidence filtering, not a semantic object classifier.
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

def segment(p,ring=None):
    if not len(p):return np.empty(0,dtype=int),np.empty(0,dtype=bool)
    distance=np.linalg.norm(p,axis=1)
    az=np.arctan2(p[:,1],p[:,0]);el=np.arctan2(p[:,2],np.hypot(p[:,0],p[:,1]))
    if ring is not None:
        group=np.argsort(ring,kind='stable');sorted_ring=ring[group]
        starts=np.r_[0,np.flatnonzero(np.diff(sorted_ring))+1];ends=np.r_[starts[1:],len(ring)]
        names=sorted_ring[starts];elev=np.array([np.median(el[group[a:b]]) for a,b in zip(starts,ends)]);order=np.argsort(elev)
        inverse=np.searchsorted(names,ring);rank=np.empty(len(names),int);rank[order]=np.arange(len(names));row=rank[inverse];height=len(names)
    else:
        row=np.rint((el-el.min())/np.deg2rad(.1)).astype(int);height=row.max()+1
    column=np.rint((az+np.pi)/np.deg2rad(.1)).astype(int)
    column-=column.min();width=int(column.max())+1
    flat=row*width+column
    order=np.lexsort((distance,flat));_,start=np.unique(flat[order],return_index=True);ids=order[start]
    grid=np.full(height*width,-1,int);grid[flat[ids]]=ids;grid=grid.reshape(height,width)
    ground=np.zeros(len(p),bool)
    low=grid[:-1].ravel();high=grid[1:].ravel();valid=(low>=0)&(high>=0);a=low[valid];b=high[valid]
    delta=p[a]-p[b];slope=np.arctan2(abs(delta[:,2]),np.linalg.norm(delta[:,:2],axis=1))
    good=(slope<np.deg2rad(10))&(p[a,2]<-.3)&(p[b,2]<-.3)
    ground[a[good]]=True;ground[b[good]]=True
    edges=[]
    for ga,gb in [(grid[:,:-1],grid[:,1:]),(grid[:-1],grid[1:])]:
        a=ga.ravel();b=gb.ravel();valid=(a>=0)&(b>=0);a=a[valid];b=b[valid]
        valid=~ground[a]&~ground[b];a=a[valid];b=b[valid]
        cos=np.clip(np.einsum('ij,ij->i',p[a],p[b])/(distance[a]*distance[b]),-1,1)
        alpha=np.arccos(cos);near=np.minimum(distance[a],distance[b]);far=np.maximum(distance[a],distance[b])
        beta=np.arctan2(near*np.sin(alpha),far-near*cos)
        valid=beta>np.deg2rad(10);edges.append(np.column_stack([a[valid],b[valid]]))
    pairs=np.concatenate(edges);graph=coo_matrix((np.ones(len(pairs)),(pairs[:,0],pairs[:,1])),shape=(len(p),len(p)))
    _,labels=connected_components(graph,directed=False)
    # Only coincident returns inherit a pixel label. A distant second return
    # must not be classified using the nearer object's surface.
    mapped=grid.ravel()[flat];coincident=np.linalg.norm(p-p[mapped],axis=1)<.03
    original_labels=labels.copy();labels[coincident]=original_labels[mapped[coincident]]
    original_ground=ground.copy();ground[coincident]=original_ground[mapped[coincident]]
    return labels,ground


def check_context(points, objects, ring=None, min_near=6, min_far=2, far_threshold=100.):
    supported=[o for o in objects if o['supported']]
    if not supported:return
    labels,ground=segment(points,ring)
    component_sizes=np.bincount(labels[~ground],minlength=len(points))
    for obj in supported:
        lo=np.asarray(obj['min']);hi=np.asarray(obj['max'])
        inside=np.all((points>=lo-1e-5)&(points<=hi+1e-5),axis=1)
        ids=np.flatnonzero(inside&~ground)
        cluster=np.unique(points[ids],axis=0)
        threshold=min_far if obj['distance_m']>=far_threshold else min_near
        reason=None
        if len(cluster)<threshold:reason='range_surface_unconfirmed'
        if len(ids):
            labs,counts=np.unique(labels[ids],return_counts=True)
            dominant=int(np.argmax(counts));coherence=float(counts[dominant]/component_sizes[labs[dominant]])
            obj['range_component_fraction']=coherence
            if counts[dominant]>=threshold and coherence<.65:reason='range_connected_background'
        obj['range_nonground_returns']=len(cluster)
        if reason:
            obj['supported']=False;obj['evidence']=reason
