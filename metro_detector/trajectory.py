"""Anchor the swept corridor to near rails and follow continuous measured walls.

An opening on one side is not a centreline displacement. Far sections extend
the near alignment only when their displacement is locally continuous.
"""
import numpy as np
from .rails import extend, uncertainty


def build_path(sections, rails, offset, plane, maximum=320., _alternative=False):
    limit=rails['fit_max_m'] if rails else 20.
    x=np.unique(np.r_[np.arange(0,maximum+1,2.),limit])
    y=extend(rails['center_coefficients'],x,limit)+offset if rails else np.full(len(x),offset)
    z=extend(rails['height_coefficients'],x,limit) if rails else plane[0]*x+plane[2]
    uy=uncertainty(rails,x,'y') if rails else np.full(len(x),.5)
    uz=uncertainty(rails,x,'z') if rails else np.full(len(x),.25)
    evidence=np.zeros(len(x),dtype=int)
    evidence[x<=limit]=1 if rails else 0
    if len(sections)>=5:
        s=np.asarray(sections,float)
        anchor=(s[:,0]>=max(8.,limit-15))&(s[:,0]<=limit)
        if anchor.sum()>=3:
            near_y=np.interp(s[anchor,0],x,y)
            offsets=np.median(s[anchor,3:5]-near_y[:,None],axis=0)
            reference_side=int(np.argmin(np.abs(offsets)))
            walls=np.column_stack([np.array([np.median(s[max(0,i-1):i+2,j]) for i in range(len(s))]) for j in (3,4)])
            measured=walls-offsets
            track_x=[limit];track_y=[float(np.interp(limit,x,y))];quality=[1];wall_error=[.15]
            slope=float(2*rails['center_coefficients'][0]*limit+rails['center_coefficients'][1]) if rails else 0.
            for i in np.flatnonzero(s[:,0]>limit):
                dx=s[i,0]-track_x[-1]
                prediction=track_y[-1]+slope*dx
                proposals=measured[i]
                both=abs(proposals[0]-proposals[1])<.6
                # Keep the same near-track wall through a bend. Picking the
                # smaller displacement at every section can switch to the
                # neighbouring track's wall and cut across the inside curve.
                target=float(np.mean(proposals)) if both else float(proposals[reference_side])
                if _alternative and not both:target=float(proposals[np.argmin(abs(proposals-prediction))])
                # Curvature can change; a sudden wall/platform jump cannot
                # redefine the route. Leave such a section unmeasured.
                if abs(target-prediction)>.18+.006*dx*dx:continue
                new_slope=(target-track_y[-1])/dx
                if abs(new_slope)>.65:continue
                track_x.append(float(s[i,0]));track_y.append(target);quality.append(2 if both else 1)
                # The selected route follows one boundary when they disagree.
                # Half the separation only bounds a midpoint, not either wall.
                wall_error.append(.15+abs(float(proposals[0]-proposals[1]))*(.5 if both else 1.))
                slope=.5*slope+.5*new_slope
            if len(track_x)>1:
                far=x>limit
                y[far]=np.interp(x[far],track_x,track_y)
                beyond=x>track_x[-1]
                y[beyond]=track_y[-1]+slope*(x[beyond]-track_x[-1])
                # Walls guide the curve but are not new rail measurements.
                # They must not erase uncertainty in rail-to-wall alignment.
                uy[far]=np.maximum(uy[far],np.interp(x[far],track_x,wall_error))
                uy[beyond]+=.025*(x[beyond]-track_x[-1])
                evidence[far]=np.interp(x[far],track_x,quality).astype(int)
                evidence[beyond]=0
            # The floor is measured independently from lateral wall offsets.
            smooth=np.array([np.median(s[max(0,i-1):i+2,1]) for i in range(len(s))])
            bias=float(np.median(smooth[anchor]-np.interp(s[anchor,0],x,z)))
            zx=[limit];zz=[float(np.interp(limit,x,z))]
            for i in np.flatnonzero(s[:,0]>limit):
                target=float(smooth[i]-bias);dx=s[i,0]-zx[-1]
                if abs(target-zz[-1])>.08+.06*dx:continue
                zx.append(float(s[i,0]));zz.append(target)
            if len(zx)>1:
                far=(x>limit)&(x<=zx[-1]);z[far]=np.interp(x[far],zx,zz);uz[far]=np.maximum(uz[far],.12)
                beyond=x>zx[-1]
                slope=np.clip((zz[-1]-zz[-2])/(zx[-1]-zx[-2]),-.06,.06)
                z[beyond]=zz[-1]+slope*(x[beyond]-zx[-1]);uz[beyond]=np.maximum(uz[beyond],.12+.015*(x[beyond]-zx[-1]))
    if rails:
        observations=np.asarray(rails['observations'])
        before=x<observations[0,0]
        if before.any():
            # No rail pair was observed here. A global quadratic can swing
            # into a side structure behind its first measurement; retain the
            # local near-rail continuation as an independent hypothesis.
            local=np.polyfit(observations[:3,0],observations[:3,1],1)
            near_y=np.polyval(local,x[before])+offset
            uy[before]=np.maximum(uy[before],.1+np.abs(y[before]-near_y))
            evidence[before]=0
        # A roof/sign can bias a cross-section floor. Disagreement with the
        # rail-head height remains an uncertainty, not a verified grade change.
        reference=extend(rails['height_coefficients'],x,limit)
        uz+=.5*np.abs(z-reference)
        # Tunnel walls alone cannot establish a new rail alignment. Preserve
        # the near-rail continuation as an independent route hypothesis.
        # A curve in the displayed walls is not proof that our track follows it.
        # This is a conservative disagreement bound, not a calibrated probability.
        rail_y=extend(rails['center_coefficients'],x,limit)+offset
        uy=np.maximum(uy,np.abs(y-rail_y))
        if len(sections)>=5:
            s=np.asarray(sections,float)
            near=(s[:,0]>=8)&(s[:,0]<=20)
            if near.sum()>=3:
                # Independently anchor the walls before the distant rail fit.
                # Otherwise a false rail pair and its wall anchor can move
                # together, leaving a confidently wrong corridor on a bend.
                near_rail=extend(rails['center_coefficients'],s[near,0],limit)+offset
                offsets=np.median(s[near,3:5]-near_rail[:,None],axis=0)
                proposals=np.column_stack([np.interp(x,s[:,0],s[:,j])-offsets[j-3] for j in (3,4)])
                disagreement=np.min(np.abs(proposals-y[:,None]),axis=1)
                observed=(x>=20)&(x<=s[-1,0])
                # An opening in just one wall is not evidence of a turn.
                uy[observed]=np.maximum(uy[observed],.15+disagreement[observed])
    if not _alternative:
        alternative=build_path(sections,rails,offset,plane,maximum,_alternative=True)
        # If maintaining the near wall and following the locally continuous
        # wall disagree, the track assignment is ambiguous. Require evidence
        # inside both corridor hypotheses; never silently choose a route.
        uy=np.maximum(uy,np.asarray(alternative['uncertainty_lateral_m']))+np.abs(y-np.asarray(alternative['lateral_m']))
    return dict(range_m=x.tolist(),lateral_m=y.tolist(),floor_m=z.tolist(),
                uncertainty_lateral_m=uy.tolist(),uncertainty_height_m=uz.tolist(),
                rail_anchored=bool(rails),evidence=evidence.tolist(),method='rail_anchored_continuous_sections')


def sample_path(path,forward,key):
    return np.interp(forward,path['range_m'],path[key])
