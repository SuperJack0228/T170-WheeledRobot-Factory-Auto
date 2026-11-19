"""Independent MDH FK comparison. No SDK/CAN/camera calls.
Usage: python reference_fk.py SOURCE_ROOT RESOLVED_3KG_RUNTIME
"""
from pathlib import Path
import csv
import math
import subprocess
import sys
import numpy as np
import yaml

root=Path(sys.argv[1]).resolve()
runtime=Path(sys.argv[2]).resolve()
actual=yaml.safe_load((runtime/'config/Robot_Model.yaml').read_text())['T7_170']
rows=[]
for kg in ('3kg','5kg'):
    model_id='t170c_v2_'+kg
    model=yaml.safe_load((root/'config/robot_models'/model_id/'Robot_Model.yaml').read_text())['T7_170']
    current=yaml.safe_load((root/'config/robot_models'/model_id/'motion_policy.yaml').read_text())
    original=yaml.safe_load(subprocess.check_output(['git','show',f'30cc366:config/robot_models/{model_id}/motion_policy.yaml'],cwd=root,text=True))
    for side in ('right','left'):
        n=model[side]
        for version,policy in [('baseline_30cc366',original),('outward_home',current)]:
            T=np.eye(4)
            q=np.deg2rad(policy[side]['home_deg'])
            for i in range(7):
                al,a,d,th=n['alpha'][i],n['a'][i],n['d'][i],q[i]+n['offset'][i]
                c,s,ca,sa=math.cos(th),math.sin(th),math.cos(al),math.sin(al)
                T=T@np.array([[c,-s,0,a],[s*ca,c*ca,-sa,-sa*d],[s*sa,c*sa,ca,ca*d],[0,0,0,1]])
                if i==3:elbow=T[:3,3].copy()
            # Same tool contract as arm_motion_policy_test: No.124's resolved
            # legacy-to-V2 flange bridge for 3 kg; nominal 260 mm for 5 kg.
            xyzrpy=actual[side]['tool'] if kg=='3kg' else [.26,0,0,0,0,0]
            x,y,z,rx,ry,rz=xyzrpy
            cx,sx,cy,sy,cz,sz=math.cos(rx),math.sin(rx),math.cos(ry),math.sin(ry),math.cos(rz),math.sin(rz)
            tool=np.eye(4)
            tool[:3,:3]=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
            tool[:3,3]=[x,y,z]
            T=T@tool
            rpy=[math.atan2(T[2,1],T[2,2]),math.atan2(-T[2,0],math.hypot(T[0,0],T[1,0])),math.atan2(T[1,0],T[0,0])]
            rows.append([model_id,side,version,policy['elbow_outward_ratio'],*policy[side]['home_deg'],*T[:3,3],*np.rad2deg(rpy),*elbow])
with (Path(__file__).parent/'home_reference_fk.csv').open('w') as stream:
    writer=csv.writer(stream)
    writer.writerow(['model','hand','policy','outward_ratio',*[f'j{i}_model_deg' for i in range(1,8)],'tcp_x_m','tcp_y_m','tcp_z_m','tcp_rx_deg','tcp_ry_deg','tcp_rz_deg','elbow_x_m','elbow_y_m','elbow_z_m'])
    writer.writerows(rows)
for i in range(0,len(rows),2):
    a,b=rows[i:i+2]
    print(b[0],b[1],'outward_mm',round((abs(b[-2])-abs(a[-2]))*1000,3),'max_abs_rpy_deg',max(abs(float(v)) for v in b[-6:-3]))
