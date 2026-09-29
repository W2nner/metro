import tempfile,unittest,time,threading,json
from pathlib import Path
import numpy as np
from metro_detector.sources import read_cloud,bag_frames,bag_info
from metro_detector.maps import read_map
from metro_detector.history import History
from metro_detector.streaming import LiveProcessor
from metro_detector.detector import Config,Detector

class IntegrationTests(unittest.TestCase):
 def test_cloud_formats_and_map_import(self):
  from plyfile import PlyData,PlyElement
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);xyz=np.array([[1,2,3],[4,5,6]],np.float32)
   (root/'points.csv').write_text('z,x,y\n3,1,2\n6,4,5\n')
   np.testing.assert_array_equal(read_cloud(root/'points.csv')[0],xyz)
   data=np.array([tuple(p) for p in xyz],dtype=[('x','f4'),('y','f4'),('z','f4')])
   for text in (True,False):
    PlyData([PlyElement.describe(data,'vertex')],text=text).write(root/'map.ply');np.testing.assert_array_equal(read_cloud(root/'map.ply')[0],xyz)
   result=read_map(root/'map.ply');self.assertEqual(result['kind'],'cloud');self.assertFalse(result['localized'])
   (root/'route.json').write_text(json.dumps({'points':[[0,0],[10,5]]}))
   self.assertEqual(read_map(root/'route.json')['points'],[[0,0,0],[10,5,0]])
   (root/'route.json').write_text(json.dumps({'points':[[0,0],[0,0]]}))
   with self.assertRaises(ValueError):read_map(root/'route.json')

 def test_bag_sqlite_and_mcap_without_ros(self):
  from rosbags.typesys import Stores,get_typestore
  from rosbags.rosbag2 import Writer,StoragePlugin
  store=get_typestore(Stores.ROS2_HUMBLE);types=store.types
  Header=types['std_msgs/msg/Header'];Time=types['builtin_interfaces/msg/Time'];Field=types['sensor_msgs/msg/PointField'];Cloud=types['sensor_msgs/msg/PointCloud2']
  xyz=np.array([[1,2,3],[4,5,6]],np.float32)
  msg=Cloud(Header(Time(1,0),'lidar'),1,2,[Field(n,i*4,7,1) for i,n in enumerate('xyz')],False,12,24,np.frombuffer(xyz.tobytes(),dtype=np.uint8),True)
  with tempfile.TemporaryDirectory() as folder:
   for plugin in (StoragePlugin.SQLITE3,StoragePlugin.MCAP):
    path=Path(folder)/plugin.name
    with Writer(path,version=8,storage_plugin=plugin) as writer:
     connection=writer.add_connection('/lidar',msg.__msgtype__,typestore=store)
     for i in range(3):writer.write(connection,1000000000+i*100000000,store.serialize_cdr(msg,msg.__msgtype__))
    for target in (path,next(path.glob('*.db3' if plugin==StoragePlugin.SQLITE3 else '*.mcap'))):
     self.assertEqual(bag_info(target),[{'topic':'/lidar','frames':3}]);rows=list(bag_frames(target));self.assertEqual(len(rows),3);np.testing.assert_array_equal(rows[0][0],xyz)
    with self.assertRaises(ValueError):list(bag_frames(path,'/missing'))

 def test_history_persistence_dedup_review_and_scope(self):
  with tempfile.TemporaryDirectory() as folder:
   p=Path(folder)/'history.db';history=History(p)
   xyz=np.array([[y,-10.,z] for z in [-1.35,-1.3,-1.25] for y in [-.15,0,.15]],np.float32)
   result=Detector(Config(ground_mode='fixed')).process(xyz,0)
   self.assertTrue(result['obstacle'])
   history.record(result,'source','key',3);history.record(result,'source','key',3)
   h=History(p);self.assertEqual(h.query()['totals']['total'],1);ident=h.query()['rows'][0]['id']
   h.review(ident,'false','box outside');self.assertEqual(h.query(verdict='false')['totals']['total'],1)
   self.assertEqual(h.get(ident)['note'],'box outside');self.assertEqual(h.query(source='other')['totals']['total'],0)
   self.assertTrue(result['obstacle'])

 def test_latest_frame_mailbox_counts_drops_and_survives_configuration(self):
  live=LiveProcessor(Config(ground_mode='fixed'))
  entered=threading.Event();release=threading.Event();original=live.detector.process
  def slow(*args,**kwargs):entered.set();release.wait(2);return original(*args,**kwargs)
  live.detector.process=slow
  try:
   cloud=np.array([[0,-10,-.5],[.1,-10,-.4]],np.float32)
   live.submit(cloud,meta={'timestamp_ns':0});self.assertTrue(entered.wait(2))
   live.submit(cloud,meta={'timestamp_ns':100000000});live.submit(cloud,meta={'timestamp_ns':200000000});self.assertEqual(live.status()['dropped'],1)
   release.set();deadline=time.monotonic()+3
   while live.status()['processed']<2 and time.monotonic()<deadline:time.sleep(.01)
   self.assertEqual(live.latest[2]['timestamp_ns'],200000000)
   live.configure(Config(ground_mode='fixed'));live.submit(cloud,meta={'timestamp_ns':300000000})
   deadline=time.monotonic()+3
   while live.latest is None and time.monotonic()<deadline:time.sleep(.01)
   self.assertEqual(live.latest[2]['timestamp_ns'],300000000)
  finally:release.set();live.close()

if __name__=='__main__':unittest.main()
