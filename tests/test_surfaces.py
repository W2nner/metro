import unittest
import numpy as np
from metro_detector.surfaces import continuing_surfaces,foreground_edges
from metro_detector.rails import track_model,extend

class SurfaceTests(unittest.TestCase):
    def item(self,p):
        return {'center':p.mean(axis=0).tolist(),'min':p.min(axis=0).tolist(),'max':p.max(axis=0).tolist(),'supported':True,'distance_m':float(np.linalg.norm(p,axis=1).min())}
    def test_floor_patch_rejected_but_low_box_preserved(self):
        floor=np.array([[x,y,-1.] for x in np.arange(8,12,.1) for y in np.arange(-2,2,.1)])
        patch=floor[(floor[:,0]>9.8)&(floor[:,0]<10.2)&(abs(floor[:,1])<.2)]
        obj=self.item(patch);continuing_surfaces(floor,[obj]);self.assertFalse(obj['supported'])
        box=patch.copy();box[:,2]+=.1;obj=self.item(box)
        continuing_surfaces(np.concatenate([floor,box]),[obj]);self.assertTrue(obj['supported'])
    def test_lateral_wall_patch_rejected(self):
        wall=np.array([[x,1.,z] for x in np.arange(8,12,.1) for z in np.arange(-1,3,.1)])
        patch=wall[(wall[:,0]>9.8)&(wall[:,0]<10.2)&(wall[:,2]>.2)&(wall[:,2]<.8)]
        obj=self.item(patch);continuing_surfaces(wall,[obj]);self.assertFalse(obj['supported'])
    def test_tangent_extension_not_unbounded_quadratic(self):
        x=np.array([10.,20.,100.]);actual=extend([.001,.02,0],x,20)
        np.testing.assert_allclose(actual,[.3,.8,5.6])
    def test_depth_edges_distinguish_foreground_from_smooth_return(self):
        rays=np.array([[np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)] for e in [-.01,.01] for a in np.arange(-.01,.011,.00175)])
        a=np.arctan2(rays[:,1],rays[:,0]);inside=abs(a)<.003
        for foreground,expected in [(True,True),(False,False)]:
            distance=np.where(inside&foreground,60.,100.)
            cloud=rays*distance[:,None];obj=self.item(cloud[inside]);obj.update(angular_support=True,geometry_support=True,unique_points=int(inside.sum()),dimensions=np.ptp(cloud[inside],axis=0).tolist())
            foreground_edges(cloud,[obj],2,2,40,required_after=40)
            self.assertEqual(obj['supported'],expected)
    def test_foreground_with_one_edge_occluded_and_shuffled_returns(self):
        rays=np.array([[np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)] for e in np.linspace(-.01,.01,8) for a in np.arange(-.01,.011,.00175)])
        az=np.arctan2(rays[:,1],rays[:,0]);inside=abs(az)<.003
        depth=np.where(inside,60.,np.where(az>.003,40.,100.))
        points=rays*depth[:,None]
        for cloud in (points,points[np.random.default_rng(8).permutation(len(points))]):
            obj=self.item(points[inside]);obj.update(supported=False,angular_support=True,geometry_support=True,unique_points=int(inside.sum()),dimensions=np.ptp(points[inside],axis=0).tolist())
            foreground_edges(cloud,[obj],6,2,100,required_after=30)
            self.assertTrue(obj['supported'])
            self.assertEqual(obj['evidence'],'foreground_partial_occlusion')
    def test_single_laser_row_depth_step_is_not_3d_evidence(self):
        angles=np.linspace(-.02,.02,60)
        rays=np.column_stack([np.cos(angles),np.sin(angles),np.full(len(angles),-.025)])
        rays/=np.linalg.norm(rays,axis=1)[:,None]
        inside=abs(angles)<.005;points=rays*np.where(inside,60.,100.)[:,None]
        obj=self.item(points[inside]);obj.update(angular_support=False,geometry_support=True,unique_points=int(inside.sum()),dimensions=np.ptp(points[inside],axis=0).tolist())
        foreground_edges(points,[obj],6,2,100,required_after=30)
        self.assertFalse(obj['supported'])

if __name__=='__main__':unittest.main()
