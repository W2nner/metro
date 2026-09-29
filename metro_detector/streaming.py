"""One processing worker, a bounded latest-frame mailbox, explicit drop counters."""
import threading
import time
import uuid
from dataclasses import asdict
from .detector import Detector

class LiveProcessor:
    def __init__(self,config,on_result=None):
        self.on_result=on_result
        self.detector=Detector(config);self.condition=threading.Condition();self.lock=threading.Lock()
        self.pending=None;self.latest=None;self.stop_event=threading.Event();self.source_stop=threading.Event()
        self.session_id=uuid.uuid4().hex
        self.received=0;self.dropped=0;self.processed=0;self.error=None;self.kind='stopped';self.generation=0
        self.thread=threading.Thread(target=self._work,daemon=True);self.thread.start();self.source_thread=None

    def submit(self,xyz,extras=None,meta=None,generation=None):
        with self.condition:
            if generation is not None and generation!=self.generation:return
            if self.pending is not None:self.dropped+=1
            self.received+=1;metadata={**(meta or {}),'session_id':self.session_id};self.pending=(xyz,extras or {},metadata,time.monotonic(),self.generation)
            self.condition.notify()

    def _work(self):
        while not self.stop_event.is_set():
            with self.condition:
                self.condition.wait_for(lambda:self.pending is not None or self.stop_event.is_set())
                if self.stop_event.is_set():break
                xyz,extra,meta,arrived,generation=self.pending;self.pending=None
            try:
                with self.lock:
                    result=self.detector.process(xyz,meta.get('timestamp_ns'),extra)
                    meta={**meta,'detector_config':asdict(self.detector.config)}
                result['source']=meta;result['queue_and_processing_ms']=(time.monotonic()-arrived)*1000
                with self.condition:
                    if generation!=self.generation:continue
                    self.processed+=1;self.latest=(self.processed,xyz,result);self.error=None
                if self.on_result:self.on_result(result,meta)
            except Exception as exc:
                with self.condition:self.error=str(exc)

    def stop_source(self):
        self.source_stop.set()
        if self.source_thread:self.source_thread.join(timeout=3)
        if self.source_thread and self.source_thread.is_alive():raise RuntimeError('Previous input is still stopping')
        with self.condition:
            self.generation+=1;self.pending=None;self.latest=None;self.kind='stopped';self.error=None
            self.received=self.dropped=self.processed=0;self.session_id=uuid.uuid4().hex
        with self.lock:self.detector.reset()
        self.source_stop=threading.Event()

    def configure(self,config):
        with self.lock:self.detector=Detector(config)
        with self.condition:self.pending=None;self.latest=None

    def status(self):
        with self.condition:return dict(kind=self.kind,received=self.received,processed=self.processed,dropped=self.dropped,error=self.error,sequence=self.latest[0] if self.latest else None)

    def start_bag(self,path,topic,rate=1.):
        from .sources import bag_info,bag_frames
        info=bag_info(path)
        if topic is None and len(info)==1:topic=info[0]['topic']
        if not any(v['topic']==topic for v in info):raise ValueError('Select a PointCloud2 topic')
        self.stop_source();self.kind='bag';stop=self.source_stop;generation=self.generation
        def play():
            try:
                first_stamp=None;started=time.monotonic()
                for xyz,extra,meta in bag_frames(path,topic):
                    if stop.is_set():break
                    if first_stamp is None:first_stamp=meta['timestamp_ns']
                    due=(meta['timestamp_ns']-first_stamp)/1e9/rate
                    if stop.wait(max(0,due-(time.monotonic()-started))):break
                    self.submit(xyz,extra,meta,generation)
                if not stop.is_set():self.kind='bag_finished'
            except Exception as exc:self.error=str(exc);self.kind='error'
        self.source_thread=threading.Thread(target=play,daemon=True);self.source_thread.start()

    def start_ros(self,topic):
        try:import rclpy
        except ImportError as exc:raise ValueError('ROS 2 not installed here. Run the Humble Docker container or use a bag file.') from exc
        if not topic.startswith('/'):raise ValueError('Use an absolute ROS topic name')
        self.stop_source();self.kind='ros';stop=self.source_stop;generation=self.generation
        def listen():
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.signals import SignalHandlerOptions
            from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
            from sensor_msgs.msg import PointCloud2
            from std_msgs.msg import String
            from .io import decode_pointcloud
            import json
            context=Context();node=None;executor=None
            try:
                rclpy.init(context=context,signal_handler_options=SignalHandlerOptions.NO)
                node=rclpy.create_node('metro_observatory',context=context)
                executor=SingleThreadedExecutor(context=context);executor.add_node(node)
                qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
                def callback(msg):
                    try:
                        xyz,extra=decode_pointcloud(msg)
                        self.submit(xyz,extra,{'timestamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,'frame_id':msg.header.frame_id,'source_topic':topic},generation)
                    except Exception as exc:self.error=str(exc)
                node.create_subscription(PointCloud2,topic,callback,qos)
                publisher=node.create_publisher(String,'/metro/obstacles',10);last=None
                while not stop.is_set():
                    executor.spin_once(timeout_sec=.02)
                    with self.condition:latest=self.latest
                    if latest is not None and latest[0]!=last:
                        last=latest[0];message=String();message.data=json.dumps(latest[2],allow_nan=False);publisher.publish(message)
            except Exception as exc:self.error=str(exc);self.kind='error'
            finally:
                if executor:executor.shutdown(timeout_sec=1)
                if node:node.destroy_node()
                if context.ok():context.shutdown()
        self.source_thread=threading.Thread(target=listen,daemon=True);self.source_thread.start()

    def close(self):
        self.stop_source();self.stop_event.set()
        with self.condition:self.condition.notify_all()
        self.thread.join(timeout=3)
