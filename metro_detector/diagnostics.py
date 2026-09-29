"""Installation and input inspection, without changing detector parameters."""
import importlib
import importlib.metadata
import json
import platform
import sys
from pathlib import Path
import numpy as np
from . import __version__
from .sources import bag_info, read_cloud, is_cloud_file


def is_bag(path):
    p=Path(path)
    return p.suffix.lower() in ('.db3','.mcap') or (p.is_dir() and (p/'metadata.yaml').is_file())


def inspect_source(path):
    p=Path(path)
    if not p.exists():raise ValueError('Input does not exist: '+str(p))
    if is_bag(p):return {'kind':'ros2_bag','topics':bag_info(p)}
    files=[p] if p.is_file() else sorted(v for v in p.iterdir() if is_cloud_file(v))
    if not files:raise ValueError('No clouds in this directory; choose a dataset subdirectory')
    xyz,extras=read_cloud(files[0]);finite=xyz[np.isfinite(xyz).all(axis=1)&np.any(xyz!=0,axis=1)]
    return {'kind':'clouds','frames':len(files),'sample':files[0].name,'input_points':len(xyz),
            'valid_points':len(finite),'fields':['x','y','z',*extras],
            'min_xyz':finite.min(axis=0).tolist() if len(finite) else None,
            'max_xyz':finite.max(axis=0).tolist() if len(finite) else None,
            'note':'Bounds of first cloud only; coordinates are metres in source basis.'}


def doctor(require_ros=False):
    checks={}
    for package in ('numpy','scipy','rosbags','plyfile'):
        try:
            importlib.import_module(package)
            checks[package]={'ok':True,'version':importlib.metadata.version(package)}
        except Exception as exc:checks[package]={'ok':False,'error':str(exc)}
    try:
        import rclpy
        from sensor_msgs.msg import PointCloud2
        checks['ros2']={'ok':True}
    except ImportError:checks['ros2']={'ok':False,'note':'Needed only for live ROS 2; files and bags work without ROS.'}
    required=list(checks) if require_ros else [k for k in checks if k!='ros2']
    return {'ok':all(checks[k]['ok'] for k in required),'version':__version__,
            'python':sys.version.split()[0],'platform':platform.platform(),'checks':checks}
