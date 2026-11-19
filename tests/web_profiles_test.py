"""Deployment file transactions and action catalog contract, without robot IO."""
from pathlib import Path
import re
import shutil
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'python')]
from web_debug.profiles import ProfileStore,read
from web_debug.actions import GROUPS


class ProfileTest(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory(prefix='t170-profiles-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)/'source'
        shutil.copytree(ROOT/'config',self.root/'config')
        self.store=ProfileStore(self.root)
        self.selected=self.store.selected()

    def payload(self,**kwargs):
        return dict(action='select',expected_manifest=self.store.selected(),manifest=self.selected,
                    robot_model='t170c_v2_5kg',**kwargs)

    def test_model_switch_validates_and_persists_next_start_only(self):
        original=(ROOT/'config/deployment.yaml').read_bytes()
        result=self.store.update(self.payload())
        self.assertTrue(result['restart_required'])
        self.assertEqual(read(self.root/self.selected)['robot_model'],'t170c_v2_5kg')
        self.assertEqual((ROOT/'config/deployment.yaml').read_bytes(),original)

    def test_new_robot_does_not_inherit_hand_eye_or_task_calibration(self):
        payload=self.payload();payload.update(action='create',robot_id='TH-T170C-0125',
                                             template=self.selected,host='192.168.192.6')
        result=self.store.update(payload)
        self.assertEqual(self.store.selected(),self.selected)
        profile=next(p for p in self.store.list()['profiles'] if p['robot_id']=='TH-T170C-0125')
        self.assertTrue(profile['missing_files'])
        self.assertEqual(profile['host'],'192.168.192.6')
        self.assertFalse((self.root/'config/calibration/TH-T170C-0125/hand_eye').exists())
        self.assertFalse(list((self.root/'config/calibration/TH-T170C-0125/tasks').glob('**/*.yaml')))
        choose=self.payload();choose['manifest']=result['manifest']
        with self.assertRaises(ValueError):self.store.update(choose)
        self.assertEqual(self.store.selected(),self.selected)
        # Complete the files in this temporary fixture, then select the new ID.
        template=read(self.root/self.selected)
        created=read(self.root/result['manifest'])
        for target,name in created['files'].items():
            destination=self.root/name
            if not destination.exists():
                destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copyfile(self.root/template['files'][target],destination)
        self.store.update(choose)
        self.assertEqual(self.store.selected(),result['manifest'])
        with self.assertRaises(ValueError):self.store.update(choose) # stale page

    def test_invalid_profile_or_model_cannot_modify_selector(self):
        before=self.store.selector.read_bytes()
        for patch in [{'manifest':'../../outside.yaml'},{'robot_model':'wrong'},
                      {'expected_manifest':'old-selection'}]:
            payload=self.payload();payload.update(patch)
            with self.assertRaises(ValueError):self.store.update(payload)
        self.assertEqual(self.store.selector.read_bytes(),before)

    def test_ui_metadata_only_refers_to_active_cpp_commands(self):
        source=(ROOT/'src/app/command_catalog.cpp').read_text()
        commands=dict(re.findall(r'\{"([^"]+)",CommandGroup::(\w+),',source))
        names=[]
        for _,_,_,entries in GROUPS:
            for name,_,_ in entries:
                self.assertIn(name,commands)
                self.assertNotEqual(commands[name],'Retired')
                names.append(name)
        self.assertEqual(len(names),len(set(names)))

if __name__=='__main__':unittest.main()
