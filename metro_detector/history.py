"""Persistent review history. Human verdicts are never detector inputs."""
import hashlib
import json
import sqlite3
import time
from contextlib import contextmanager
from pathlib import Path

class History:
    def __init__(self,path):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        with self.connect() as db:
            db.execute('PRAGMA journal_mode=WAL')
            db.execute('''CREATE TABLE IF NOT EXISTS detections (
                id INTEGER PRIMARY KEY, fingerprint TEXT UNIQUE NOT NULL,
                recorded_at REAL NOT NULL, source TEXT NOT NULL, source_key TEXT NOT NULL,
                frame INTEGER, timestamp_ns INTEGER, distance REAL, mode TEXT,
                confirmed INTEGER, object_count INTEGER, latency REAL, payload TEXT NOT NULL,
                verdict TEXT NOT NULL DEFAULT 'unreviewed', note TEXT NOT NULL DEFAULT '')''')

    @contextmanager
    def connect(self):
        db=sqlite3.connect(self.path,timeout=10)
        try:
            with db:yield db
        finally:db.close()

    def record(self,result,source,source_key,frame=None,config=None):
        if not result.get('obstacle'):return
        objects=[o for o in result['objects'] if o['supported'] and (result['mode']=='single' or o['confirmed'])]
        geometry=[{k:o[k] for k in ('min','max','evidence')} for o in objects]
        key=json.dumps([source_key,frame,result.get('timestamp_ns'),config or {},result.get('distance_m'),geometry],sort_keys=True)
        fingerprint=hashlib.sha256(key.encode()).hexdigest()
        payload=json.dumps({'result':result,'config':config or {}},allow_nan=False)
        with self.connect() as db:
            db.execute('''INSERT INTO detections
                (fingerprint,recorded_at,source,source_key,frame,timestamp_ns,distance,mode,confirmed,object_count,latency,payload)
                VALUES (?,?,?,?,?,?,?,?,?,?,?,?)
                ON CONFLICT(fingerprint) DO UPDATE SET payload=CASE WHEN excluded.confirmed>=detections.confirmed THEN excluded.payload ELSE detections.payload END, confirmed=MAX(detections.confirmed,excluded.confirmed)''',
                (fingerprint,time.time(),source,source_key,frame,result.get('timestamp_ns'),result['distance_m'],result['mode'],
                 int(any(o['confirmed'] for o in objects)),len(objects),result['processing_ms'],payload))
            return db.execute('SELECT id FROM detections WHERE fingerprint=?',(fingerprint,)).fetchone()[0]

    def query(self,source='',verdict='',confirmed=False,limit=100,offset=0):
        clauses=[];args=[]
        if source:clauses.append('source=?');args.append(source)
        if verdict:
            if verdict not in ('unreviewed','true','false'):raise ValueError('Invalid verdict')
            clauses.append('verdict=?');args.append(verdict)
        if confirmed:clauses.append('confirmed=1')
        where=(' WHERE '+' AND '.join(clauses)) if clauses else ''
        with self.connect() as db:
            db.row_factory=sqlite3.Row
            totals=dict(db.execute('SELECT COUNT(*) total,MIN(distance) nearest,SUM(confirmed) confirmed,SUM(verdict=\'false\') false_count FROM detections'+where,args).fetchone())
            rows=[dict(r) for r in db.execute('SELECT id,recorded_at,source,frame,timestamp_ns,distance,mode,confirmed,object_count,latency,verdict,note FROM detections'+where+' ORDER BY recorded_at DESC,id DESC LIMIT ? OFFSET ?',args+[min(1000,max(1,limit)),max(0,offset)])]
            sources=[r[0] for r in db.execute('SELECT DISTINCT source FROM detections ORDER BY source')]
            bins=[dict(r) for r in db.execute('SELECT CAST(distance/25 AS INTEGER)*25 start_m,COUNT(*) count FROM detections'+where+' GROUP BY start_m ORDER BY start_m',args)]
        return dict(rows=rows,totals=totals,sources=sources,histogram=bins)

    def get(self,ident):
        with self.connect() as db:
            db.row_factory=sqlite3.Row;row=db.execute('SELECT * FROM detections WHERE id=?',(ident,)).fetchone()
        if row is None:raise ValueError('History entry not found')
        value=dict(row);value.update(json.loads(value.pop('payload')));return value

    def review(self,ident,verdict,note=''):
        if verdict not in ('unreviewed','true','false'):raise ValueError('Invalid verdict')
        if not isinstance(note,str) or len(note)>2000:raise ValueError('Note must be at most 2000 characters')
        with self.connect() as db:
            cursor=db.execute('UPDATE detections SET verdict=?,note=? WHERE id=?',(verdict,note,ident))
            if not cursor.rowcount:raise ValueError('History entry not found')
