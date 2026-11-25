from pathlib import Path
import copy
import sys
import tempfile
import unittest
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from t170_common.conveyor_heights import load_catalog
from t170_vision.geometry.conveyor import api,detector


class ConveyorIdentityTest(unittest.TestCase):
    def setUp(self):
        self.cfg=detector.load_config(ROOT/'config/sites/current_factory/vision/belt.yaml')

    def test_catalog_contains_all_ten_markers(self):
        self.assertTrue(self.cfg.height_catalog_enabled)
        self.assertEqual(self.cfg.all_ids,{500,*range(550,560)})
        self.assertEqual([(b.right_id,b.machine_id) for b in self.cfg.belts],
                         [(550+i,str(i//2+1)+('.1' if i%2 else '.2')) for i in range(10)])

    def test_selection_does_not_fall_back_to_wrong_machine_or_first_result(self):
        half={'name':'YF-0003','machine_id':'2.2','operation_profile':'conveyor'}
        finish={'name':'YF-0004','machine_id':'2.1','operation_profile':'conveyor2'}
        self.assertEqual(api.select_belt([finish,half],'belt0',True)[0],half)
        self.assertEqual(api.select_belt([half,finish],'belt1',True)[0],finish)
        self.assertIsNone(api.select_belt([finish],'belt0',True)[0])
        self.assertIsNone(api.select_belt([half,dict(half,name='YF-0005',machine_id='3.2')],'belt0',True)[0])
        self.assertEqual(api.select_belt([half,finish],'YF-0004',True)[0],finish)

    def test_deployed_workcell_preferences_select_the_correct_machine_type(self):
        workcells=yaml.safe_load((ROOT/'config/sites/current_factory/workcell_profiles.yaml').read_text())
        for group in range(1,6):
            found=[{'name':'half','machine_id':str(group)+'.2','operation_profile':'conveyor'},
                   {'name':'finish','machine_id':str(group)+'.1','operation_profile':'conveyor2'}]
            for role in ('conveyor','conveyor2'):
                chosen,message=api.select_belt(found,workcells[role]['prefer_belt'],True)
                self.assertIsNotNone(chosen,message)
                self.assertEqual(chosen['operation_profile'],role)

    def test_real_aruco_image_maps_to_machine(self):
        api.init(ROOT/'config/sites/current_factory/vision/belt.yaml')
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        frame=np.full((720,1280,3),255,np.uint8)
        for x,code in [(400,500),(800,554)]:
            marker=cv2.aruco.generateImageMarker(dictionary,code,86)
            frame[250:336,x:x+86]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
        result=api.detect_frame(frame,np.array([[900,0,640],[0,900,360],[0,0,1]]),np.zeros(5),
                                cam2robot=np.eye(4),prefer_name='belt0')
        self.assertTrue(result['ok'],result['message'])
        self.assertEqual(result['name'],'YF-0005');self.assertIn(554,result['used_ids'])
        self.assertFalse(api.detect_frame(frame,np.array([[900,0,640],[0,900,360],[0,0,1]]),np.zeros(5),prefer_name='belt1')['ok'])

    def test_public_location_marker_alone_has_no_identity(self):
        K=np.array([[900.,0,640],[0,900.,360],[0,0,1.]])
        corners=[np.array([[[10,10],[20,10],[20,20],[10,20]]],np.float32)]
        for belt in self.cfg.belts:
            self.assertIsNone(detector.estimate_belt_pose(corners,np.array([[500]]),self.cfg,belt,K,np.zeros(5)))

    def test_duplicate_shared_location_markers_do_not_corrupt_pose(self):
        belt=self.cfg.belts[0];K=np.array([[900.,0,640],[0,900.,360],[0,0,1.]])
        corners=[]
        for mid in (500,500,550):
            obj=detector.marker_corners_in_belt(self.cfg,belt,mid)
            points,_=cv2.projectPoints(obj,np.zeros(3),np.array([.1,.02,.8]),K,np.zeros(5))
            corners.append(points.reshape(1,4,2).astype(np.float32))
        pose=detector.estimate_belt_pose(corners,np.array([[500],[500],[550]]),self.cfg,belt,K,np.zeros(5))
        self.assertIsNotNone(pose);self.assertEqual(pose.used_ids,[550])


if __name__=='__main__':unittest.main()
