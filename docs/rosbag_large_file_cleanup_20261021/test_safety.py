import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from audit import signature, sha
import safe_delete as sd

class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='rosbag_cleanup_test_')
        self.root=Path(self.temp.name);self.target=self.root/'copy.bag';self.keeper=self.root/'original.bag'
        for p in [self.target,self.keeper]:p.write_bytes(b'test content: not a real bag')
        self.digest=sha(self.keeper)
        self.records={str(p):dict(sha256=sha(p),signature=signature(p)) for p in [self.target,self.keeper]}
        self.entry=dict(path=str(self.target),retained_copy=str(self.keeper),classification='SAFE_TO_DELETE',
                        basis='A_EXACT_SHA_DUPLICATE',sha256=self.digest)
    def tearDown(self):self.temp.cleanup()
    def test_explicit_duplicate_only(self):
        events=[]
        with patch.object(sd,'unused',return_value=[]):sd.delete_one(self.entry,self.records,self.root,{},journal=events.append)
        self.assertFalse(self.target.exists());self.assertEqual(sha(self.keeper),self.digest)
        self.assertEqual([e['phase'] for e in events],['INTENT','UNLINKED'])
    def test_protected_rejected(self):
        with self.assertRaises(RuntimeError):sd.validate_pair(self.target,self.keeper,self.digest,self.records,self.root,{str(self.target):self.digest})
    def test_outside_root_rejected(self):
        with self.assertRaises(RuntimeError):sd.confined_file(self.root.parent,self.root)
    def test_symlink_rejected(self):
        link=self.root/'link';link.symlink_to(self.target)
        with self.assertRaises(RuntimeError):sd.confined_file(link,self.root)
    def test_same_path_rejected(self):
        with self.assertRaises(RuntimeError):sd.validate_pair(self.target,self.target,self.digest,self.records,self.root,{})
    def test_same_inode_rejected(self):
        link=self.root/'hardlink';os.link(self.target,link)
        self.records[str(link)]=dict(sha256=self.digest,signature=signature(link))
        self.records[str(self.target)]['signature']=signature(self.target)
        with self.assertRaises(RuntimeError):sd.validate_pair(self.target,link,self.digest,self.records,self.root,{})
    def test_changed_file_rejected(self):
        self.target.write_bytes(b'changed')
        with self.assertRaises(RuntimeError):sd.validate_pair(self.target,self.keeper,self.digest,self.records,self.root,{})
    def test_different_hash_rejected(self):
        self.records[str(self.target)]['sha256']='0'*64
        with self.assertRaises(RuntimeError):sd.validate_pair(self.target,self.keeper,self.digest,self.records,self.root,{})
    def test_uncertain_rejected(self):
        self.entry['classification']='UNCERTAIN'
        with self.assertRaises(RuntimeError):sd.delete_one(self.entry,self.records,self.root,{})
    def test_other_process_blocks(self):
        output=type('Result',(),dict(returncode=0,stdout='9999999',stderr=''))()
        with patch.object(sd.subprocess,'run',return_value=output):
            with self.assertRaises(RuntimeError):sd.unused([self.target])
    def test_process_check_error_blocks(self):
        output=type('Result',(),dict(returncode=1,stdout='',stderr='Cannot inspect processes'))()
        with patch.object(sd.subprocess,'run',return_value=output):
            with self.assertRaises(RuntimeError):sd.unused([self.target])
    def test_no_unsupported_fuser_separator(self):
        calls=[]
        def result(argv,**kwargs):
            calls.append(argv)
            return type('Result',(),dict(returncode=1,stdout='',stderr=''))()
        with patch.object(sd.subprocess,'run',side_effect=result):sd.unused([self.target])
        self.assertEqual(calls[0][:2],['fuser','-I']);self.assertNotIn('--',calls[0])
    def test_ancestor_mount_rejected(self):
        nested=self.root/'nested';nested.mkdir();target=nested/'copy.bag';target.write_bytes(b'test')
        with patch.object(sd.os.path,'ismount',side_effect=lambda p:Path(p)==nested):
            with self.assertRaises(RuntimeError):sd.confined_file(target,self.root)
    def test_same_device_bind_mount_rejected(self):
        with patch.object(sd,'mountpoints',return_value={self.root}),patch.object(sd.os.path,'ismount',return_value=False):
            with self.assertRaises(RuntimeError):sd.confined_file(self.target,self.root)
    def test_self_pid_plus_fuser_warning_rejected(self):
        output=type('Result',(),dict(returncode=0,stdout=str(os.getpid()),stderr=str(self.target)+': Permission denied while scanning'))()
        with patch.object(sd.subprocess,'run',return_value=output):
            with self.assertRaises(RuntimeError):sd.unused([self.target])
    def test_journal_failure_prevents_unlink(self):
        def failure(record):raise OSError('journal unavailable')
        with patch.object(sd,'unused',return_value=[]):
            with self.assertRaises(OSError):sd.delete_one(self.entry,self.records,self.root,{},journal=failure)
        self.assertTrue(self.target.exists())
    def test_missing_journal_prevents_unlink(self):
        with patch.object(sd,'unused',return_value=[]):
            with self.assertRaises(RuntimeError):sd.delete_one(self.entry,self.records,self.root,{})
        self.assertTrue(self.target.exists())
    def test_post_unlink_fsync_failure_has_intent_and_outcome(self):
        events=[]
        with patch.object(sd,'unused',return_value=[]),patch.object(sd.os,'fsync',side_effect=OSError('fsync failed')):
            with self.assertRaises(OSError):sd.delete_one(self.entry,self.records,self.root,{},journal=events.append)
        self.assertFalse(self.target.exists());self.assertEqual([e['phase'] for e in events],['INTENT','UNLINKED'])

if __name__=='__main__':unittest.main()
