"""Create a clean handoff; never include state, raw datasets or work files."""
import argparse
import hashlib
import json
import shutil
import subprocess
import zipfile
from pathlib import Path

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('output');parser.add_argument('--bundle-name',default='METRO-SUBMISSION-2026-09-29.zip');parser.add_argument('--audit',default='reports/release-linux');args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    audit_path=root/args.audit
    audit=json.loads((audit_path/'report.json').read_text())
    assert audit['complete'] and sum(d['frames'] for d in audit['datasets'].values())==13759
    assert all(digest(root/'metro_detector'/k)==v for k,v in audit['detector_sha256'].items())
    image=json.loads((out/'IMAGE_INFO.json').read_text())
    current=json.loads(subprocess.check_output(['docker','image','inspect','metro-detector:submission']))[0]
    assert image['Id']==current['Id'],'Exported image is not current'
    files=[]
    for folder in ('metro_detector','config','web','launch','examples','docs','tests'):
        files.extend(p for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc')
    for name in ('README.md','START_HERE.md','QUICKSTART.md','THIRD_PARTY_NOTICES.md','Dockerfile','.dockerignore','.gitignore','.gitattributes','requirements.txt','requirements-container.txt','compose.yaml','start-panel.cmd','run-panel.ps1','start.sh'):
        files.append(root/name)
    for name in ('evaluate_all.py','ros_smoke.py','package_submission.py','audit_alarm_windows.py','inject_real_rays.py'):
        files.append(root/'tools'/name)
    for name in ('RELEASE_VALIDATION.md','release-validation-log.txt','performance-v13.json','real-ray-injection.json','person-crossing-review.json','RESEARCH_V12.md'):
        files.append(root/'reports'/name)
    files.extend(p for p in (root/'reports/release-linux').glob('*') if p.suffix in ('.json','.jsonl') and '.progress.' not in p.name)
    if audit_path.resolve()!=(root/'reports/release-linux').resolve():
        files.extend(p for p in audit_path.glob('*') if p.suffix in ('.json','.jsonl') and '.progress.' not in p.name)
    for name in ('release-turns-0.4.2','release-linux-0.4.2'):
        files.extend(p for p in (root/'reports'/name).glob('*') if p.suffix in ('.json','.jsonl') and '.progress.' not in p.name)
    files.extend(p for p in (root/'reports').glob('*0.4.2*') if p.is_file() and p.suffix in ('.md','.json','.txt'))
    files=list(dict.fromkeys(files))
    manifest={str(p.relative_to(root)).replace('\\','/'):digest(p) for p in sorted(files)}
    (out/'SOURCE_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    for source,target in [('START_HERE.md','START_HERE.md'),('reports/RELEASE_VALIDATION.md','RELEASE_VALIDATION.md')]:
        shutil.copyfile(root/source,out/target)
    with zipfile.ZipFile(out/'metro-detector-source.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,p.relative_to(root).as_posix())
        archive.write(out/'SOURCE_MANIFEST.json','SOURCE_MANIFEST.json')
        archive.write(out/'VALIDATION.json','VALIDATION.json')
    artifacts=['metro-detector-source.zip','metro-detector-image.tar.gz','METRO-overview.mp4','START_HERE.md','RELEASE_VALIDATION.md','IMAGE_INFO.json','VALIDATION.json','SOURCE_MANIFEST.json']
    checksums={name:digest(out/name) for name in artifacts}
    (out/'SHA256SUMS.txt').write_text(''.join(f'{value}  {name}\n' for name,value in checksums.items()),encoding='utf-8')
    bundle=out.parent/args.bundle_name
    with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,'metro-detector/'+p.relative_to(root).as_posix())
        for name in ['SOURCE_MANIFEST.json','VALIDATION.json','IMAGE_INFO.json','METRO-overview.mp4','metro-detector-image.tar.gz']:
            archive.write(out/name,'metro-detector/'+name,compress_type=zipfile.ZIP_STORED if name.endswith(('.gz','.mp4')) else zipfile.ZIP_DEFLATED)
    with zipfile.ZipFile(bundle) as archive:assert archive.testzip() is None
    bundle.with_suffix('.sha256').write_text(digest(bundle)+'  '+bundle.name+'\n',encoding='utf-8')
    print(json.dumps({'bundle':str(bundle),'bytes':bundle.stat().st_size,'source_files':len(files)},indent=2))

if __name__=='__main__':main()
