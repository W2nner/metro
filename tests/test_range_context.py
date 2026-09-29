import unittest
import numpy as np
from metro_detector.range_context import segment,check_context
from metro_detector.detector import Detector,Config


class RangeContextTests(unittest.TestCase):
    def test_second_return_does_not_inherit_first_surface(self):
        points=np.array([[10.,0.,-1.],[20.,0.,-2.],[10.,0.,-1.]])
        labels,ground=segment(points,np.array([17,17,17]))
        self.assertEqual(labels[0],labels[2])
        self.assertNotEqual(labels[0],labels[1])

    def test_sparse_noncontiguous_ring_ids_and_order(self):
        points=np.array([[10.,0.,-1.],[10.,.017,-1.],[10.,0.,-.8],[10.,.017,-.8]])
        ring=np.array([2000,2000,7,7]);labels,_=segment(points,ring)
        permutation=np.array([3,0,2,1]);other,_=segment(points[permutation],ring[permutation])
        np.testing.assert_array_equal(labels[:,None]==labels, (other[:,None]==other)[np.argsort(permutation)][:,np.argsort(permutation)])

    def test_empty_projection_and_bad_ring_length(self):
        labels,ground=segment(np.empty((0,3)))
        self.assertEqual(len(labels),0)
        with self.assertRaises(ValueError):Detector().process(np.ones((4,3)),extras={'ring':[1,2]})

    def test_small_vertical_face_survives_ring_context(self):
        # 30 x 10 cm frontal face. No class/size whitelist is used.
        xyz=np.array([[y,-10.,z] for z in [-1.35,-1.3,-1.25] for y in [-.15,0,.15]],dtype=np.float32)
        ring=np.repeat([70,19,125],3)
        result=Detector(Config(ground_mode='fixed',ground_z_m=-1.4)).process(xyz,extras={'ring':ring})
        self.assertTrue(result['obstacle'])

if __name__=='__main__':unittest.main()
