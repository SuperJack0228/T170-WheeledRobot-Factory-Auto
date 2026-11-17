#!/usr/bin/env python3
"""Generate a selected robot's runtime tree. Never opens hardware."""
from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from t170_common.deployment import resolve
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('manifest',type=Path);p.add_argument('output',type=Path);p.add_argument('--root',type=Path,default=ROOT)
    a=p.parse_args()
    try:
        r=resolve(a.root,a.manifest,a.output);print(json.dumps({'robot_id':r['robot_id'],'robot_model':r['robot_model'],'runtime_root':r['runtime_root'],'task_assignment':r['task_assignment'],'arm_model_prepared':'selected_arm_model' in r},ensure_ascii=False))
    except (ValueError,OSError) as e:p.exit(2,str(e)+'\n')
