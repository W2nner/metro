"""End-to-end DDS test using a padded PointCloud2 and the production node."""
import subprocess,time,json,sys,tempfile,os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
rclpy.init();node=Node('metro_smoke_test');results=[]
sub=node.create_subscription(String,'/metro/obstacles',lambda msg:results.append(json.loads(msg.data)),10)
pub=node.create_publisher(PointCloud2,'/test_points',qos_profile_sensor_data)
# This fixture contains only an object, with no rails/tunnel to calibrate from.
# Use explicit calibration so this tests DDS/decoding rather than inventing
# reliable geometry from eight points.
fixture=tempfile.NamedTemporaryFile(mode='w',suffix='.json',delete=False)
json.dump({'ground_mode':'fixed','ground_z_m':-1.4,'calibration_verified':True},fixture);fixture.close()
command=['ros2','launch','/app/launch/metro.launch.py','topic:=/test_points','config:='+fixture.name] if '--launch' in sys.argv else [sys.executable,'-m','metro_detector','ros','--topic','/test_points','--config',fixture.name]
process=subprocess.Popen(command,start_new_session=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
try:
    msg=PointCloud2();msg.header.frame_id='test_lidar';msg.height=1;msg.width=8;msg.point_step=16;msg.row_step=128;msg.is_dense=True
    msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
    data=np.zeros((8,4),dtype='<f4');data[:,0]=np.linspace(0,.1,8);data[:,1]=-160;data[:,2]=np.linspace(-.7,-.5,8);msg.data=data.tobytes()
    deadline=time.monotonic()+20
    while time.monotonic()<deadline and not results:
        msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg);rclpy.spin_once(node,timeout_sec=.2)
    assert results,'No DDS detection output'
    assert results[0]['obstacle'] and 159<results[0]['distance_m']<161,results[0]
    print('ROS2 DDS PASS:',results[0]['status'],results[0]['distance_m'])
finally:
    import signal
    # ros2 launch forwards SIGINT to its child; signalling the whole group
    # would interrupt the child's cleanup a second time.
    process.send_signal(signal.SIGINT)
    try:log,_=process.communicate(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);log,_=process.communicate()
    print(log)
    node.destroy_node();rclpy.shutdown();os.unlink(fixture.name)
    assert process.returncode==0,('Unclean shutdown',process.returncode,log)
    assert 'process has died' not in log and 'terminate called' not in log,log
