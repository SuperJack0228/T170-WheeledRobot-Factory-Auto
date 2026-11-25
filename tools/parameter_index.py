#!/usr/bin/env python3
"""Find effective parameter values and their source files; never edits source configuration."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from t170_common.deployment import resolve

def value_at(config,key):
    for part in key.split('.'):
        config=config[part]
    return config

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('query',nargs='?',default='',help='Parameter name or substring, e.g. visual_servo.kp or descend')
    parser.add_argument('--deployment',type=Path,help='Explicit deployment; otherwise use config/deployment.yaml')
    args=parser.parse_args()
    try:
        manifest=args.deployment
        if manifest is None:
            selector=yaml.safe_load((ROOT/'config/deployment.yaml').read_text())
            manifest=ROOT/'config'/selector['manifest']
        # Reuse the real resolver and ownership validation; this temporary overlay
        # performs no SDK/device IO and makes no changes to canonical calibration.
        with tempfile.TemporaryDirectory(prefix='t170-parameter-index-') as temporary:
            runtime=Path(temporary)/'runtime'
            record=resolve(ROOT,manifest,runtime)
            config=yaml.safe_load((runtime/'config/effective_params.yaml').read_text())
            print(f"robot={record['robot_id']} model={record['robot_model']} deployment={manifest.resolve()}")
            selected=[key for key in sorted(record['parameter_sources']) if args.query.lower() in key.lower()]
            for key in selected:
                print(key+' = '+json.dumps(value_at(config,key),ensure_ascii=False))
                print('  source: '+str(ROOT/record['parameter_sources'][key]))
            if not selected:
                print('No matching merged control parameter. SDK, camera SN, hand-eye and vision resources are selected by the deployment files/model/robot profiles.',file=sys.stderr)
                return 1
        return 0
    except (OSError,ValueError,KeyError,yaml.YAMLError) as ex:
        print(str(ex),file=sys.stderr);return 2

if __name__=='__main__':
    raise SystemExit(main())
