import unittest
from types import SimpleNamespace as NS
import numpy as np
from metro_detector.detector import Config, Detector, basis,connected_cells
from metro_detector.io import decode_pointcloud

class GeometryTests(unittest.TestCase):
    def test_fast_voxel_graph_matches_bruteforce(self):
        rng=np.random.default_rng(42)
        for _ in range(10):
            points=rng.uniform(-3,3,(80,3));cells=np.floor(points/.6).astype(int)
            remaining=set(range(len(points)));expected=[]
            while remaining:
                group={remaining.pop()};queue=list(group)
                while queue:
                    i=queue.pop();adjacent={j for j in remaining if np.max(np.abs(cells[i]-cells[j]))<=1}
                    remaining-=adjacent;group|=adjacent;queue.extend(adjacent)
                expected.append(frozenset(group))
            actual=[frozenset(a.tolist()) for a in connected_cells(points,.6)]
            self.assertEqual(set(actual),set(expected))
    def config(self,**kw):return Config(ground_mode='fixed',ground_z_m=-1.4,**kw)
    def object(self,x=160,y=0):
        return np.array([[y,-x,-.8],[y+.05,-x-.1,-.7],[y+.1,-x-.1,-.6]],np.float32)
    def test_long_range_nearest_point(self):
        xyz=self.object();r=Detector(self.config()).process(xyz,0)
        self.assertTrue(r['obstacle']);self.assertAlmostEqual(r['distance_m'],float(np.linalg.norm(xyz[0])),places=4)
    def test_margin_and_mask(self):
        xyz=self.object(y=1.1)
        self.assertFalse(Detector(self.config()).process(xyz)['obstacle'])
        self.assertTrue(Detector(self.config(margin_enabled=True)).process(xyz)['obstacle'])
        mask={'min':[150,-2,-2],'max':[170,2,2]}
        self.assertFalse(Detector(self.config(margin_enabled=True,self_masks=[mask])).process(xyz)['obstacle'])
    def test_confirmation_and_timestamp_reset(self):
        d=Detector(self.config(mode='confirmed'))
        self.assertFalse(d.process(self.object(),0)['obstacle'])
        self.assertTrue(d.process(self.object(158),100000000)['obstacle'])
        self.assertFalse(d.process(self.object(158),100000000)['obstacle'])
        self.assertFalse(d.process(self.object(),3000000000)['obstacle'])
    def test_finite_zero_empty(self):
        r=Detector(self.config()).process(np.array([[0,0,0],[np.nan,0,0],[np.inf,1,1]],np.float32))
        self.assertEqual(r['valid_points'],0);self.assertFalse(r['obstacle'])
    def test_endian_and_row_padding(self):
        data=bytearray(64)
        dtype=np.dtype({'names':['x','y','z'],'formats':['>f4']*3,'offsets':[0,4,8],'itemsize':12})
        rows=np.ndarray((2,2),dtype=dtype,buffer=data,strides=(32,12))
        rows['x']=[[1,2],[3,4]];rows['y']=7;rows['z']=-2
        msg=NS(is_bigendian=True,fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')],point_step=12,row_step=32,height=2,width=2,data=data)
        xyz,_=decode_pointcloud(msg)
        np.testing.assert_array_equal(xyz[:,0],[1,2,3,4]);np.testing.assert_array_equal(xyz[:,1],7)
    def test_axes(self):
        np.testing.assert_array_equal(basis(self.config())@np.array([2,-10,3]),[10,2,3])
    def test_low_small_obstacle_retained(self):
        xyz=np.array([[x,-160+f,-1.3+z] for x in [0,.1,.2] for f in [0,.1,.2] for z in [0,.05]],np.float32)
        self.assertTrue(Detector(self.config()).process(xyz)['obstacle'])
    def test_duplicate_returns_are_not_independent_evidence(self):
        xyz=np.tile(self.object()[0],(20,1))
        result=Detector(self.config()).process(xyz)
        self.assertFalse(result['obstacle']);self.assertEqual(result['status'],'UNCERTAIN')
        self.assertEqual(result['objects'][0]['unique_points'],1)

if __name__=='__main__':unittest.main()
