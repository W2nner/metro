import csv
import heapq
import json
import sqlite3
from pathlib import Path
import numpy as np

FIELD_TYPES={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}

def read_pcd(path):
    with Path(path).open('rb') as f:
        header={}
        for _ in range(100):
            line=f.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key,*values=line.split();header[key]=values
                if key=='DATA':break
        names=header['FIELDS']; counts=[int(c) for c in header.get('COUNT',['1']*len(names))]
        formats=[('<'+{'F':'f','I':'i','U':'u'}[t]+size) for t,size in zip(header['TYPE'],header['SIZE'])]
        dtype=np.dtype([(n,fmt) if count==1 else (n,fmt,(count,)) for n,fmt,count in zip(names,formats,counts)])
        if header['DATA']==['binary']:
            data=np.frombuffer(f.read(),dtype=dtype)
        elif header['DATA']==['ascii']:
            data=np.loadtxt(f,dtype=dtype,ndmin=1)
        else:raise ValueError('Supported PCD encodings: binary, ascii (not binary_compressed)')
        if len(data)!=int(header['POINTS'][0]):raise ValueError('PCD payload length mismatch')
    xyz=np.column_stack([data[n] for n in ('x','y','z')]).astype(np.float32,copy=False)
    extras={n:data[n] for n in ('intensity','ring') if n in names}
    return xyz,extras

def decode_pointcloud(msg):
    endian='>' if msg.is_bigendian else '<'
    fields=[f for f in msg.fields if f.name in ('x','y','z','intensity','ring')]
    if not {'x','y','z'} <= {f.name for f in fields}:raise ValueError('Missing XYZ')
    if any(f.count!=1 for f in fields):raise ValueError('XYZ/intensity/ring must be scalar')
    dtype=np.dtype({'names':[f.name for f in fields],'formats':[endian+FIELD_TYPES[f.datatype] for f in fields],
                    'offsets':[f.offset for f in fields],'itemsize':msg.point_step})
    data=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
    xyz=np.column_stack([data[n] for n in ('x','y','z')]).astype(np.float32,copy=False)
    return xyz,{n:data[n] for n in ('intensity','ring') if n in data.dtype.names}

def bag_messages(path,topic=None):
    """Merge lightweight SQLite indices; load only one serialized message at a time."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    path=Path(path).resolve()
    dbs=sorted(path.glob('*.db3')) if path.is_dir() else [path]
    if not dbs or any(db.suffix!='.db3' or not db.is_file() for db in dbs):
        raise ValueError('Expected a ROS2 SQLite3 bag directory or .db3 file; MCAP is not supported by this reader')
    connections=[]; streams=[]
    try:
        available=set()
        for db in dbs:
            with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as probe:
                available.update(r[0] for r in probe.execute('SELECT name FROM topics WHERE type=?',('sensor_msgs/msg/PointCloud2',)))
        if topic is None:
            if len(available)!=1:raise ValueError('Specify --topic; PointCloud2 topics: '+str(sorted(available)))
            topic=next(iter(available))
        if topic not in available:raise ValueError('PointCloud2 topic not found: '+topic)
        for i,db in enumerate(dbs):
            con=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True);connections.append(con)
            sql='SELECT m.timestamp,m.id,t.name FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.type=?'
            params=['sensor_msgs/msg/PointCloud2']
            if topic:sql+=' AND t.name=?';params.append(topic)
            cursor=con.execute(sql+' ORDER BY m.timestamp,m.id',params)
            def stream(cursor=cursor,i=i):
                for stamp,ident,name in cursor:yield stamp,i,ident,name
            streams.append(stream())
        for index,(stamp,db_index,ident,name) in enumerate(heapq.merge(*streams)):
            raw=connections[db_index].execute('SELECT data FROM messages WHERE id=?',(ident,)).fetchone()[0]
            msg=deserialize_message(raw,PointCloud2)
            xyz,extra=decode_pointcloud(msg)
            yield xyz,extra,{'frame_index':index,'timestamp_ns':stamp,'header_timestamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                              'frame_id':msg.header.frame_id,'source_topic':name,'source_db':dbs[db_index].name,'source_message_id':ident}
    finally:
        for con in connections:con.close()

def pcd_sequence(path):
    path=Path(path)
    if path.is_file():yield (*read_pcd(path),{'frame_index':0,'filename':str(path)});return
    mappings={}
    if (path/'frames.csv').exists():
        with (path/'frames.csv').open(encoding='utf-8') as f:mappings={r['pcd_filename']:r for r in csv.DictReader(f)}
    for i,p in enumerate(sorted(path.glob('*.pcd'))):
        row=mappings.get(p.name,{})
        yield (*read_pcd(p),{'frame_index':i,'filename':str(p),'timestamp_ns':int(row.get('timestamp',i*100000000)),
                            'frame_id':row.get('frame_id','lidar')})
