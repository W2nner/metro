import json, subprocess, sys, tempfile, unittest
from pathlib import Path
import numpy as np
from metro_detector.sources import read_cloud

ROOT=Path(__file__).resolve().parents[1]
class ReleaseTests(unittest.TestCase):
    def cli(self,*args):
        return subprocess.run([sys.executable,'-m','metro_detector',*args],cwd=ROOT,capture_output=True,text=True,timeout=15)
    def test_cli_jsonl_summary_and_stdout(self):
        with tempfile.TemporaryDirectory() as d:
            dest=Path(d)/'nested/result.jsonl';summary=Path(d)/'summary.json'
            result=self.cli('process','examples/obstacle.xyz','--config','examples/fixture-config.json','--output',str(dest),'--summary',str(summary))
            self.assertEqual(result.returncode,0,result.stderr)
            rows=[json.loads(v) for v in dest.read_text().splitlines()]
            self.assertEqual(len(rows),1);self.assertTrue(rows[0]['obstacle'])
            self.assertEqual(json.loads(summary.read_text())['alarm_frames'],1)
            stdout=self.cli('process','examples/obstacle.xyz','--config','examples/fixture-config.json','--output','-')
            self.assertEqual(stdout.returncode,0,stdout.stderr);self.assertTrue(json.loads(stdout.stdout)['obstacle'])
    def test_source_overwrite_and_invalid_limit_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'source.xyz';original='0 -10 -1\n';p.write_text(original)
            r=self.cli('process',str(p),'--output',str(p))
            self.assertNotEqual(r.returncode,0);self.assertEqual(p.read_text(),original)
            r=self.cli('process',str(p),'--output',str(Path(d)/'out.jsonl'),'--limit','-1')
            self.assertNotEqual(r.returncode,0)
    def test_inspection_and_csv_fields(self):
        r=self.cli('inspect','examples/obstacle.xyz')
        self.assertEqual(r.returncode,0,r.stderr);self.assertEqual(json.loads(r.stdout)['input_points'],9)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'cloud.csv';p.write_text('z, x, y, ring, intensity\n3,1,2,4,5\n')
            xyz,extra=read_cloud(p);np.testing.assert_array_equal(xyz,[[1,2,3]])
            np.testing.assert_array_equal(extra['ring'],[4]);np.testing.assert_array_equal(extra['intensity'],[5])
    def test_doctor(self):
        r=self.cli('doctor');self.assertEqual(r.returncode,0,r.stderr)
        self.assertTrue(json.loads(r.stdout)['ok'])

if __name__=='__main__':unittest.main()
