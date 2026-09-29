"""Headless ROS 2 entry uses the same bounded worker as the dashboard."""
import time
from .streaming import LiveProcessor

def run(topic,config):
    processor=LiveProcessor(config)
    try:
        processor.start_ros(topic)
        print('PointCloud2 '+topic+' -> /metro/obstacles; latest-frame queue, depth 1',flush=True)
        while processor.source_thread.is_alive():time.sleep(.2)
        if processor.error:raise RuntimeError(processor.error)
    except KeyboardInterrupt:pass
    finally:processor.close()
