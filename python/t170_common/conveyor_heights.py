"""Site-owned machine identities and measured QR-plane heights."""
from pathlib import Path
import math
import yaml


def load_catalog(path):
    data=yaml.safe_load(Path(path).read_text())
    if data.get('schema_version')!=1 or type(data.get('enabled')) is not bool:
        raise ValueError('Invalid conveyor height schema/enabled')
    rows=data.get('machines')
    if not isinstance(rows,list):raise ValueError('machines must be a sequence')
    names=set();businesses=set();information=set();locations=set()
    for row in rows:
        name=str(row['machine_id']);business=str(row['business_id'])
        locate=row['locate_id'];info=row['info_id'];height=row['qr_plane_height_cm']
        if (not name or not business or name in names or business in businesses or info in information or
            type(locate) is not int or type(info) is not int or not 0<=locate<1000 or not 0<=info<1000 or locate==info or
            row['operation_profile'] not in ('conveyor','conveyor2') or type(height) not in (int,float) or
            not math.isfinite(height) or height<=0):
            raise ValueError('Invalid/duplicate conveyor height record: '+name)
        row['machine_id']=name;row['business_id']=business
        names.add(name);businesses.add(business);information.add(info);locations.add(locate)
    if information&locations:raise ValueError('Information ID conflicts with location ID')
    if data.get('reference_machine') not in names:raise ValueError('Unknown conveyor reference machine')
    return data
