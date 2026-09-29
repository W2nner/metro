import unittest
import numpy as np
from metro_detector.trajectory import build_path,sample_path


class TrajectoryTests(unittest.TestCase):
    def rail(self):
        return dict(fit_max_m=30.,center_coefficients=[0,0,0],height_coefficients=[0,0,-1.4],
                    observations=[[x,0,-1.4,1.6,0] for x in np.arange(4.5,31,3)])

    def test_curve_follows_measured_wall_despite_opposite_platform(self):
        sections=[]
        for x in np.arange(3.,123.,3.):
            center=.0015*max(0,x-30)**2
            sections.append([x,-1.4,3.,center-3,center+3+(4 if 36<x<110 else 0),200])
        path=build_path(sections,self.rail(),0,np.array([0,0,-1.4]))
        for x in (45,60,90):
            self.assertAlmostEqual(float(sample_path(path,x,'lateral_m')),.0015*(x-30)**2,delta=.15)
        self.assertGreater(float(sample_path(path,90,'lateral_m')),4)

    def test_wall_bend_does_not_verify_a_new_route(self):
        sections=[[x,-1.4,3.,.0015*max(0,x-30)**2-3,.0015*max(0,x-30)**2+3,200] for x in np.arange(3.,123.,3.)]
        path=build_path(sections,self.rail(),0,np.array([0,0,-1.4]))
        self.assertGreater(float(sample_path(path,90,'uncertainty_lateral_m')),4.)
        self.assertTrue(path['rail_anchored'])
        unanchored=build_path(sections,None,0,np.array([0,0,-1.4]))
        self.assertFalse(unanchored['rail_anchored'])

    def test_single_wall_opening_does_not_pull_straight_track(self):
        sections=[[x,-1.4,3.,-3,3+(5 if x>35 else 0),200] for x in np.arange(3.,123.,3.)]
        path=build_path(sections,self.rail(),0,np.array([0,0,-1.4]))
        self.assertLess(np.max(np.abs(path['lateral_m'])),.01)

    def test_bend_keeps_near_track_wall_not_neighbour_track(self):
        sections=[]
        for x in np.arange(3.,123.,3.):
            center=.0015*max(0,x-30)**2
            # The remote wall belongs to another track and gradually shifts.
            remote_shift=-min(1.5,max(0,x-35)*.035)
            sections.append([x,-1.4,3.,center-3,center+7+remote_shift,200])
        path=build_path(sections,self.rail(),0,np.array([0,0,-1.4]))
        self.assertAlmostEqual(float(sample_path(path,100,'lateral_m')),7.35,delta=.15)
        self.assertGreater(float(sample_path(path,100,'uncertainty_lateral_m')),.8)

    def test_missing_sections_increase_extrapolation_uncertainty(self):
        sections=[[x,-1.4,3.,-3,3,200] for x in np.arange(3.,61.,3.)]
        path=build_path(sections,self.rail(),0,np.array([0,0,-1.4]))
        self.assertGreater(float(sample_path(path,150,'uncertainty_lateral_m')),1.)
        self.assertEqual(path['evidence'][path['range_m'].index(150.)],0)

    def test_walls_cannot_erase_rail_alignment_uncertainty(self):
        from metro_detector.rails import uncertainty
        rail=self.rail()
        sections=[[x,-1.4,3.,-3,3,200] for x in np.arange(3.,303.,3.)]
        path=build_path(sections,rail,0,np.array([0,0,-1.4]))
        x=np.asarray(path['range_m'])
        self.assertTrue(np.all(np.asarray(path['uncertainty_lateral_m'])>=uncertainty(rail,x,'y')-1e-9))

    def test_conflicting_walls_bound_selected_side_not_midpoint(self):
        sections=[[x,-1.4,3.,-3.,3.+(.9 if x>35 else 0),200] for x in np.arange(3.,123.,3.)]
        path=build_path(sections,self.rail(),0,np.array([0,0,-1.4]))
        self.assertAlmostEqual(float(sample_path(path,60,'lateral_m')),0.,delta=.01)
        self.assertGreaterEqual(float(sample_path(path,60,'uncertainty_lateral_m')),1.04)

    def test_wall_disagreement_inside_rail_fit_is_not_confident(self):
        rail=self.rail();rail['center_coefficients']=[-.0015,0,0]
        rail['observations']=[[x,-.0015*x*x,-1.4,1.6,0] for x in np.arange(4.5,31,3)]
        sections=[[x,-1.4,3.,-3.,3.,200] for x in np.arange(3.,93.,3.)]
        path=build_path(sections,rail,0,np.array([0,0,-1.4]))
        self.assertGreater(float(sample_path(path,30,'uncertainty_lateral_m')),.9)

    def test_backward_extrapolation_retains_local_rail_hypothesis(self):
        rail=self.rail();rail['center_coefficients']=[-.0027,.15,-.73]
        rail['observations']=[[x,float(np.polyval(rail['center_coefficients'],x)),-1.4,1.6,0] for x in [7.5,10.5,19.5,22.5,25.5,28.5]]
        path=build_path([],rail,0,np.array([0,0,-1.4]))
        self.assertEqual(sample_path(path,2,'evidence'),0)
        self.assertGreater(float(sample_path(path,2,'uncertainty_lateral_m')),.25)

    def test_consistent_curved_rails_and_walls_keep_central_clearance(self):
        rail=self.rail();rail['center_coefficients']=[.0015,0,0]
        rail['observations']=[[x,.0015*x*x,-1.4,1.6,0] for x in np.arange(4.5,31,3)]
        sections=[[x,-1.4,3.,.0015*x*x-3,.0015*x*x+3,200] for x in np.arange(3.,93.,3.)]
        path=build_path(sections,rail,0,np.array([0,0,-1.4]))
        self.assertAlmostEqual(float(sample_path(path,28,'lateral_m')),.0015*28**2,delta=.02)
        self.assertLess(float(sample_path(path,28,'uncertainty_lateral_m')),.25)


if __name__=='__main__':unittest.main()
