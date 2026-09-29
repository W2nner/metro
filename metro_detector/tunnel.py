"""Single-scan cross sections; no stored tunnel map or per-recording parameters."""
import numpy as np

def profile(points, offset=0.):
    sections=[]
    if len(points)<200:return sections
    edges=[2.]
    while edges[-1]<350:edges.append(edges[-1]+max(3.,edges[-1]*.08))
    order=np.argsort(points[:,0]);ordered=points[order]
    bounds=np.searchsorted(ordered[:,0],edges)
    for start,end,i,j in zip(edges[:-1],edges[1:],bounds[:-1],bounds[1:]):
        s=ordered[i:j]
        if len(s)<24:continue
        lower,upper=np.quantile(s[:,2],[.04,.98])
        nearfloor=s[(np.abs(s[:,1]-offset)<1.3)&(s[:,2]<lower+.65)]
        # Estimate running-surface height from the upper low surface, not the
        # dense bed/drain below the rail heads. Never fit to tall candidates.
        floor=float(np.quantile(nearfloor[:,2],.98)) if len(nearfloor)>=16 else float(lower+.2)
        left,right=np.quantile(s[:,1],[.01,.99])
        sections.append([float(np.median(s[:,0])),floor,float(upper),float(left),float(right),len(s)])
    if len(sections)>5:
        a=np.asarray(sections)
        reliable=(a[:,0]>12)&(a[:,0]<50)&(a[:,2]-a[:,1]>3.4)&(a[:,2]-a[:,1]<6)
        if reliable.sum()>=3:
            tunnel_height=float(np.median(a[reliable,2]-a[reliable,1]))
            # Missing floor returns must not raise the estimated running surface
            # into the tunnel roof. Geometry is estimated independently per scan.
            a[:,1]=np.minimum(a[:,1],a[:,2]-tunnel_height)
            # Near-field ceiling is outside the vertical field of view.
            a[a[:,0]<12,1]=np.asarray(sections)[a[:,0]<12,1]
        return a.tolist()
    return sections

def interpolate(points,sections,fallback):
    if len(sections)<3:return np.full(len(points),fallback),np.full(len(points),np.inf),np.full(len(points),-np.inf),np.full(len(points),np.inf)
    s=np.asarray(sections)
    return tuple(np.interp(points[:,0],s[:,0],s[:,i]) for i in range(1,5))


def centerline(points,sections,offset,interpolate_points=True):
    if len(sections)<5:return np.full(len(points),offset),None
    s=np.asarray(sections);near=(s[:,0]>8)&(s[:,0]<20)
    if near.sum()<2:return np.full(len(points),offset),None
    # Extrapolate only the near-field mounting alignment to the sensor origin.
    # Far geometry follows measured sections. Compare both boundaries so a
    # platform/second-track opening does not drag the corridor into that opening.
    shifts=[]
    for side in (3,4):
        coefficient=np.polyfit(s[near,0],s[near,side],1)
        anchor=float(coefficient[1]);values=s[:,side].copy()
        values=np.array([np.median(values[max(0,i-1):i+2]) for i in range(len(values))])
        shifts.append(values-anchor)
    left,right=shifts
    trace=np.where(np.abs(left-right)<.6,(left+right)/2,np.where(np.abs(left)<np.abs(right),left,right))+offset
    return (np.interp(points[:,0],s[:,0],trace) if interpolate_points else np.full(len(points),offset)),{'range_m':s[:,0].tolist(),'lateral_m':trace.tolist(),'boundary':'paired_common_displacement'}
