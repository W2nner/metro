"""Portable cloud and ROS 2 bag input; ROS installation is not needed for files."""
from pathlib import Path
import numpy as np
from .io import read_pcd, decode_pointcloud

CLOUD_SUFFIXES={'.pcd','.ply','.xyz','.csv'}

def is_cloud_file(path):
    path=Path(path)
    if not path.is_file() or path.name=='frames.csv':return False
    if path.suffix.lower() in ('.pcd','.ply','.xyz'):return True
    if path.suffix.lower()!='.csv':return False
    try:
        with path.open(encoding='utf-8-sig') as f:columns=f.readline(4096).strip().split(',')
        if {'x','y','z'}<=set(v.strip().lower() for v in columns):return True
        return len(columns)>=3 and all(np.isfinite(float(v)) for v in columns[:3])
    except (OSError,UnicodeError,ValueError):return False


def read_cloud(path):
    path=Path(path)
    suffix=path.suffix.lower()
    if suffix=='.pcd':return read_pcd(path)
    if suffix=='.ply':
        from plyfile import PlyData
        data=PlyData.read(path)['vertex'].data
        if not {'x','y','z'}<=set(data.dtype.names):raise ValueError('PLY needs vertex x/y/z')
        xyz=np.column_stack([data[n] for n in ('x','y','z')]).astype(np.float32)
        extra={n:np.asarray(data[n]) for n in ('ring','intensity') if n in data.dtype.names}
    elif suffix in ('.xyz','.csv'):
        # XYZ has whitespace; CSV accepts a header x,y,z or plain numeric rows.
        with path.open(encoding='utf-8-sig') as f:first=f.readline()
        delimiter=',' if suffix=='.csv' else None
        names=[v.strip() for v in first.strip().lower().split(',')] if delimiter else first.strip().lower().split()
        header={'x','y','z'}<=set(names)
        data=np.loadtxt(path,delimiter=delimiter,skiprows=int(header),ndmin=2,encoding='utf-8-sig')
        columns=[names.index(n) for n in ('x','y','z')] if header else [0,1,2]
        if data.shape[1]<3:raise ValueError('Cloud needs at least three XYZ columns')
        xyz=data[:,columns].astype(np.float32)
        extra={n:data[:,names.index(n)] for n in ('ring','intensity') if header and n in names}
    else:raise ValueError('Supported clouds: PCD, PLY, XYZ, CSV')
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not len(xyz):raise ValueError('Empty or invalid cloud')
    return xyz,extra

def bag_info(path):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores,get_typestore
    with AnyReader([Path(path)],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        return [{'topic':name,'frames':sum(c.msgcount for c in reader.connections if c.topic==name)}
                for name in sorted({c.topic for c in reader.connections if c.msgtype=='sensor_msgs/msg/PointCloud2'})]

def bag_frames(path,topic=None):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores,get_typestore
    with AnyReader([Path(path)],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        choices={c.topic for c in reader.connections if c.msgtype=='sensor_msgs/msg/PointCloud2'}
        if topic is None:
            if len(choices)!=1:raise ValueError('Choose PointCloud2 topic: '+', '.join(sorted(choices)))
            topic=next(iter(choices))
        if topic not in choices:raise ValueError('PointCloud2 topic not found: '+topic)
        connections=[c for c in reader.connections if c.topic==topic and c.msgtype=='sensor_msgs/msg/PointCloud2']
        for index,(connection,stamp,data) in enumerate(reader.messages(connections=connections)):
            msg=reader.deserialize(data,connection.msgtype)
            xyz,extra=decode_pointcloud(msg)
            yield xyz,extra,{'frame_index':index,'timestamp_ns':stamp,'frame_id':msg.header.frame_id,'source_topic':topic}

def cloud_sequence(path):
    import csv
    path=Path(path)
    if path.is_file():yield (*read_cloud(path),{'frame_index':0,'filename':path.name});return
    files=sorted(p for p in path.iterdir() if is_cloud_file(p))
    stamps={}
    if (path/'frames.csv').exists():
        with (path/'frames.csv').open(encoding='utf-8') as f:stamps={r['pcd_filename']:int(r['timestamp']) for r in csv.DictReader(f)}
    if not files:raise ValueError('No supported clouds found')
    for index,p in enumerate(files):yield (*read_cloud(p),{'frame_index':index,'filename':p.name,'timestamp_ns':stamps.get(p.name,index*100000000)})
