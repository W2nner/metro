"""Optional display maps, independent from obstacle decisions."""
import json
from pathlib import Path
import numpy as np
from .sources import read_cloud

def read_map(path):
    path=Path(path)
    if path.suffix.lower() in ('.json','.geojson'):
        doc=json.loads(path.read_text(encoding='utf-8-sig'))
        if doc.get('type')=='FeatureCollection':
            geometries=[v.get('geometry',{}) for v in doc['features'] if v.get('geometry',{}).get('type')=='LineString']
            if len(geometries)!=1:raise ValueError('GeoJSON must contain one LineString route')
            doc=geometries[0]
        elif doc.get('type')=='Feature':doc=doc['geometry']
        geographic=doc.get('type')=='LineString'
        points=np.asarray(doc.get('coordinates') if geographic else doc.get('points'),dtype=float)
        if points.ndim!=2 or points.shape[1] not in (2,3):raise ValueError('Route must be Nx2 or Nx3')
        if points.shape[1]==2:points=np.c_[points,np.zeros(len(points))]
        if geographic and len(points):
            origin=points[0].copy();points[:,0]=(points[:,0]-origin[0])*111320*np.cos(np.deg2rad(origin[1]));points[:,1]=(points[:,1]-origin[1])*111320;points[:,2]-=origin[2]
        kind='route';note='GeoJSON projected to a local metre grid' if geographic else 'Local coordinates, metres'
    else:
        points,_=read_cloud(path);kind='cloud' if path.suffix.lower() in ('.pcd','.ply') else 'route';note='Local coordinates, metres'
    if not len(points) or not np.isfinite(points).all():raise ValueError('Map contains no points or nonfinite coordinates')
    if kind=='route':
        if len(points)<2 or len(points)>200000:raise ValueError('Route needs 2..200000 points')
        if np.linalg.norm(np.diff(points,axis=0),axis=1).sum()<.01:raise ValueError('Route has zero length')
    points=points[::max(1,int(np.ceil(len(points)/(200000 if kind=='route' else 120000))))]
    return dict(kind=kind,points=points.tolist(),name=path.name,note=note,demo=False,localized=False)
