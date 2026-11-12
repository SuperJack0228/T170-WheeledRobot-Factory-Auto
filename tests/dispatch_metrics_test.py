import json
from pathlib import Path
import subprocess
import sys
import tempfile
with tempfile.TemporaryDirectory() as tmp:
    subprocess.run([sys.argv[1],tmp],check=True,capture_output=True)
    files=list((Path(tmp)/'logs/dispatch_metrics').glob('run_*.jsonl'))
    assert len(files)==1,files
    events=[json.loads(line) for line in files[0].read_text().splitlines()]
    assert events[0]['type']=='run_start' and events[-1]['type']=='run_end'
    assert [e['seq'] for e in events]==list(range(1,len(events)+1))
    starts=[e for e in events if e['type']=='mission_start']
    span=next(e for e in events if e['type']=='span')
    assert span['mission_id']==starts[0]['mission_id'], 'span was misattributed to next mission'
    assert span['step']=='parallel step'
    assert starts[0]['mission_id']!=starts[1]['mission_id']
    assert events[-1]['metrics']['dropped_events']==0
    assert all(e['wall'].endswith('Z') for e in events)
    assert len([e for e in events if e['type']=='mission_end'])==2
with tempfile.TemporaryDirectory() as tmp:
    subprocess.run([sys.argv[1],tmp,'off'],check=True,capture_output=True)
    assert not (Path(tmp)/'logs/dispatch_metrics').exists(), 'T170_TIMING=0 created telemetry'
print('async dispatch writer, unique IDs, JSON escaping, captured span context and clean flush passed')
