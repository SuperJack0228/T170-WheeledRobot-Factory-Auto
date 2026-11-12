import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
spec=importlib.util.spec_from_file_location('report',Path(__file__).resolve().parents[1]/'tools/dispatch_report.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
class ReportTests(unittest.TestCase):
    def event(self,kind,mid='',**kw):return dict(type=kind,run_id='run',mission_id=mid,mission='LOAD',metrics={},**kw)
    def test_denominators_retries_incomplete_and_parallel(self):
        events=[self.event('run_start'),self.event('run_end',metrics_override=True)]
        events[-1]['metrics']={'dur_ms':3600000,'dropped_events':2}
        for mid in ['a','b','c','d']:events.append(self.event('mission_start',mid))
        for mid,status in [('a','success'),('b','success'),('d','failed')]:
            events.append(self.event('mission_end',mid,status=status));events[-1]['metrics']={'dur_ms':1000}
        events+=[self.event('retry','b'),self.event('part_count','a',name='tray2_placed')];events[-1]['metrics']={'count':2}
        summary,missions,_,_=r.summarize(events)
        self.assertEqual(summary['by_type']['LOAD']['completion_rate_all_started'],.5)
        self.assertEqual(summary['by_type']['LOAD']['first_pass_rate'],.25)
        self.assertEqual(summary['by_type']['LOAD']['incomplete'],1)
        self.assertEqual(summary['reported_placed_per_hour_closed_runs'],2)
        self.assertIsNone(summary['unattended_success_rate'])
        self.assertEqual(summary['dropped_events'],2)
    def test_copy_dedup_truncated_json_and_html_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);p=root/'run.jsonl'
            events=[self.event('mission_start','a',seq=1),self.event('mission_end','a',seq=2,status='failed',detail='<script>alert(1)</script>')]
            p.write_text('\n'.join(json.dumps(e) for e in events)+'\n{"truncated"',encoding='utf-8')
            summary=r.export_report([p,p],root/'out')
            self.assertEqual(summary['missions'],1);self.assertEqual(len(summary['parse_warnings']),2)
            doc=(root/'out/report.html').read_text()
            self.assertNotIn('<script>alert',doc);self.assertIn('&lt;script&gt;',doc)
            self.assertTrue((root/'out/missions.csv').is_file())
if __name__=='__main__':unittest.main()
