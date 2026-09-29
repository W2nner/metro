"""Estimate track center and rail-head plane from paired longitudinal ridges."""
import numpy as np
from itertools import combinations
from scipy.signal import find_peaks

def extend(coefficients,x,limit):
    a,b,c=coefficients
    near=np.minimum(x,limit)
    return a*near*near+b*near+c+(x-near)*(2*a*limit+b)

def uncertainty(model,x,coordinate):
    observations=np.asarray(model['observations'])
    A=np.column_stack([observations[:,0]**2,observations[:,0],np.ones(len(observations))])
    residual=observations[:,1 if coordinate=='y' else 2]-A@np.asarray(model['center_coefficients' if coordinate=='y' else 'height_coefficients'])
    variance=max(.025**2,float(np.sum(residual**2)/max(1,len(residual)-3)))
    covariance=np.linalg.pinv(A.T@A)*variance
    limit=model['fit_max_m'];near=np.minimum(x,limit)
    design=np.column_stack([near**2+2*near*(x-near),x,np.ones(len(x))])
    return .02+2*np.sqrt(np.maximum(0,np.einsum('ij,jk,ik->i',design,covariance,design)))

def track_model(points):
    observations=[]
    for start in np.arange(3,34,3):
        s=points[(points[:,0]>=start)&(points[:,0]<start+3)&(np.abs(points[:,1])<5)]
        if len(s)<100:continue
        lower=np.quantile(s[:,2],.05)
        s=s[(s[:,2]<lower+.7)&(s[:,2]>lower-.3)]
        if len(s)<60:continue
        cell=np.floor((s[:,1]+5)/.04).astype(int);valid=(cell>=0)&(cell<250);s=s[valid];cell=cell[valid]
        order=np.lexsort((s[:,2],cell));sorted_cells=cell[order]
        starts=np.r_[0,np.flatnonzero(np.diff(sorted_cells))+1];ends=np.r_[starts[1:],len(order)]
        values=np.full(250,lower-.2)
        enough=ends-starts>=4
        quantile=starts+np.floor((ends-starts-1)*.9).astype(int)
        values[sorted_cells[starts[enough]]]=s[order[quantile[enough]],2]
        peaks,properties=find_peaks(values,prominence=.08,distance=5)
        pairs=[]
        x=start+1.5
        predicted=0.
        if observations:
            predicted=observations[-1][1]
            if len(observations)>1:predicted+=(observations[-1][1]-observations[-2][1])/(observations[-1][0]-observations[-2][0])*(x-observations[-1][0])
        for i in peaks:
            for j in peaks:
                gauge=(j-i)*.04
                if not 1.35<gauge<1.76 or abs(values[i]-values[j])>.24:continue
                center=(i+j)/2*.04-5+.02
                if observations and abs(center-predicted)>.35:continue
                score=abs(center-predicted)+abs(gauge-1.6)*2+abs(values[i]-values[j])
                pairs.append((score,center,(values[i]+values[j])/2,gauge,(values[j]-values[i])/gauge))
        if pairs:
            _,y,z,g,cross=min(pairs);observations.append([x,float(y),float(z),float(g),float(cross)])
    if len(observations)<6:return None
    a=np.array(observations)
    # A few distant troughs can resemble rails. Retain a joint, continuous
    # centre/height consensus instead of abandoning all good near observations.
    samples=np.asarray(list(combinations(range(len(a)),3)),dtype=int)
    samples=samples[np.ptp(a[samples,0],axis=1)>=12]
    if not len(samples):return None
    x=a[samples,0];design=np.stack([x*x,x,np.ones_like(x)],axis=2)
    coefficients=np.linalg.solve(design,a[samples,1:3])
    good=(np.abs(coefficients[:,0,:])<=.003).all(axis=1)
    coefficients=coefficients[good]
    if not len(coefficients):return None
    full_design=np.column_stack([a[:,0]**2,a[:,0],np.ones(len(a))])
    error=np.max(np.abs(np.einsum('ij,kjl->kil',full_design,coefficients)-a[None,:,1:3]),axis=2)
    keep=error<.10;counts=keep.sum(axis=1);cost=np.minimum(error,.10).sum(axis=1)
    winner=int(np.lexsort((np.arange(len(counts)),cost,-counts))[0]);best=(None,keep[winner])
    if best[1].sum()<max(6,int(np.ceil(len(a)*.7))):return None
    a=a[best[1]]
    if np.ptp(a[:,0])<15 or a[0,0]>10:return None
    observations=a.tolist()
    y=np.polyfit(a[:,0],a[:,1],2);z=np.polyfit(a[:,0],a[:,2],2)
    ey=float(np.max(np.abs(np.polyval(y,a[:,0])-a[:,1])));ez=float(np.max(np.abs(np.polyval(z,a[:,0])-a[:,2])))
    if ey>.15 or ez>.15 or abs(y[0])>.003 or abs(z[0])>.003:return None
    return {'center_coefficients':y.tolist(),'height_coefficients':z.tolist(),'cross_slope':float(np.median(a[:,4])),
            'observations':observations,'fit_max_m':float(a[-1,0]),'residual_y_m':ey,'residual_z_m':ez,'gauge_m':float(np.median(a[:,3]))}
