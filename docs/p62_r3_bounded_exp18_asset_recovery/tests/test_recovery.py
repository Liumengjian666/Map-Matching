import hashlib
import io
import json
import shutil
import sys
import tempfile
import time
import unittest
from contextlib import ExitStack, redirect_stdout
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import recovery_io as rio
import recover as app


class Response:
    def __init__(self,body,range_value,status=206,fail=False):
        self.status=status;self.headers={'Content-Range':range_value}
        self.body=io.BytesIO(body);self.fail=fail
    def __enter__(self):return self
    def __exit__(self,*args):return False
    def read1(self,n):
        if self.fail and self.body.tell()>0:raise TimeoutError('synthetic socket timeout')
        return self.body.read(min(n,3) if self.fail else n)


class Opener:
    def __init__(self,files,fail_segment=False,bad_prefix=False):
        self.files=files;self.calls=[];self.fail_segment=fail_segment;self.bad_prefix=bad_prefix
    def open(self,req,timeout):
        self.calls.append(req);assert timeout==90
        start,end=map(int,req.headers['Range'][6:].split('-'))
        key='a.bag' if 'a.bag' in req.full_url else 'b.e57'
        payload=self.files[key];body=payload[start:end+1]
        if self.bad_prefix and start==0:body=b'X'+body[1:]
        return Response(body,f'bytes {start}-{end}/{len(payload)}',
                        fail=self.fail_segment and start>0)


class Guards(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='p62_r3_test_',dir='/home/jian/livox_ws')
        self.base=Path(self.temp.name)
        self.stack=ExitStack();self.stack.enter_context(patch.object(rio,'BASE',self.base))
    def tearDown(self):self.stack.close();self.temp.cleanup()
    def request(self,response):
        class Fixed:
            def open(self,*args,**kwargs):return response
        return rio.request_bytes('https://huggingface.co/test',0,3,4,time.time()+30,{},Fixed())
    def test_exact_range(self):self.assertEqual(self.request(Response(b'abcd','bytes 0-3/4')),b'abcd')
    def test_reject_200(self):
        with self.assertRaises(rio.IdentityError):self.request(Response(b'abcd','bytes 0-3/4',200))
    def test_reject_wrong_range(self):
        with self.assertRaises(rio.IdentityError):self.request(Response(b'abcd','bytes 1-4/5'))
    def test_reject_short_body(self):
        with self.assertRaises(rio.TransportError):self.request(Response(b'ab','bytes 0-3/4'))
    def test_reject_excess_body(self):
        with self.assertRaises(rio.IdentityError):self.request(Response(b'abcde','bytes 0-3/4'))
    def test_symlink_rejected(self):
        p=self.base/'link';p.symlink_to('/tmp')
        with self.assertRaises(rio.IdentityError):rio.internal(p/'file')
    def test_deadline(self):
        with self.assertRaises(rio.DeadlineError):rio.check_deadline(time.time()-1)
    def test_reserve(self):
        with patch.object(rio.shutil,'disk_usage',return_value=shutil._ntuple_diskusage(100,1,21)):
            with self.assertRaises(rio.StorageError):rio.reserve(self.base,30,20)
    def test_prefix_conflict_no_write(self):
        p=self.base/'data';p.write_bytes(b'abc')
        with self.assertRaises(rio.IdentityError):rio.append_complete(p,b'new',2,time.time()+10)
        self.assertEqual(p.read_bytes(),b'abc')
    def test_publish_bad_hash_no_link(self):
        p=self.base/'data';p.write_bytes(b'abc');dst=self.base/'verified'
        with self.assertRaises(rio.IdentityError):rio.publish(p,dst,3,'0'*64,b'a',time.time()+10)
        self.assertFalse(dst.exists())
    def test_publish_and_no_overwrite(self):
        p=self.base/'data';p.write_bytes(b'abc');dst=self.base/'verified';sha=hashlib.sha256(b'abc').hexdigest()
        result=rio.publish(p,dst,3,sha,b'a',time.time()+10)
        self.assertTrue(result['hardlink_verified'])
        with self.assertRaises(rio.IdentityError):rio.publish(p,dst,3,sha,b'a',time.time()+10)
    def setup_run(self,fail=False,bad_prefix=False,missing_entry=False):
        old=self.base/'old';(old/'staging').mkdir(parents=True)
        doc=self.base/'docs';doc.mkdir()
        linear=old/'staging'/'a.bag.part';linear.write_bytes(b'#ROSBAG V2.0\nseed')
        files={'a.bag':linear.read_bytes()+b'0123456789abcdef', 'b.e57':b'ASTM-E57abcdefghij'}
        frozen=self.base/'frozen.json'
        frozen.write_text(json.dumps({'preserved_incomplete_files':[] if missing_entry else [rio.identity(linear)]}))
        fake=Opener(files,fail,bad_prefix)
        for name,value in {'BASE':self.base,'OLD':old,'ROOT':self.base/'new','DOC':doc,
             'LINEAR':linear,'SEED_SIZE':linear.stat().st_size,'PREFIX_BYTES':13,'CHUNK':8,
             'FROZEN_FILE':frozen,'FROZEN_SHA':rio.identity(frozen)['sha256'],
             'DEADLINE':time.time()+120,'FILES':[(name,len(body),hashlib.sha256(body).hexdigest(),
                      b'#ROSBAG V2.0\n' if name=='a.bag' else b'ASTM-E57') for name,body in files.items()]}.items():
            self.stack.enter_context(patch.object(app,name,value))
        self.stack.enter_context(patch.object(app.urllib.request,'build_opener',return_value=fake))
        # Synthetic capacity; no real network and tiny fixtures only.
        self.stack.enter_context(patch.object(shutil,'disk_usage',return_value=shutil._ntuple_diskusage(100000000000,0,48000000000)))
        return fake,linear,doc
    def run_quiet(self):
        with redirect_stdout(io.StringIO()):return app.run()
    def test_full_success_bag_then_e57(self):
        fake,linear,doc=self.setup_run();before=rio.identity(linear)
        result=self.run_quiet()
        self.assertEqual(result['result'],'ASSET_RECOVERY_PASS_BOTH_FIXED_REVISION_FILES')
        self.assertEqual(before,rio.identity(linear))
        self.assertIn('b.e57',fake.calls[-1].full_url)
    def test_two_attempts_no_partial_append(self):
        fake,linear,doc=self.setup_run(fail=True);seed=linear.read_bytes()
        result=self.run_quiet()
        self.assertEqual(result['result'],'ASSET_RECOVERY_BLOCKED_TRANSPORT')
        self.assertEqual(len(fake.calls),3)
        self.assertEqual((app.ROOT/'staging/a.bag.recovering.part').read_bytes(),seed)
        self.assertEqual(fake.calls[1].headers['Range'],fake.calls[2].headers['Range'])
    def test_prefix_mismatch_stops(self):
        fake,linear,doc=self.setup_run(bad_prefix=True)
        self.assertEqual(self.run_quiet()['result'],'ASSET_RECOVERY_BLOCKED_IDENTITY')
        self.assertEqual(len(fake.calls),1)
        self.assertEqual(list((app.ROOT/'staging').iterdir()),[])
    def test_missing_frozen_linear_fails_closed(self):
        fake,linear,doc=self.setup_run(missing_entry=True)
        self.assertEqual(self.run_quiet()['result'],'ASSET_RECOVERY_BLOCKED_IDENTITY')
        self.assertFalse(fake.calls)
    def test_start_capacity_below_threshold(self):
        fake,linear,doc=self.setup_run()
        with patch.object(shutil,'disk_usage',return_value=shutil._ntuple_diskusage(100000000000,0,47999999999)):
            self.assertEqual(self.run_quiet()['result'],'ASSET_RECOVERY_BLOCKED_STORAGE')
        self.assertFalse(fake.calls)
    def test_finalization_preserves_primary_failure(self):
        fake,linear,doc=self.setup_run(missing_entry=True)
        original=app.old_snapshot;calls=[]
        def snapshot(deadline):
            calls.append(1)
            if len(calls)==2:raise OSError('synthetic final snapshot failure')
            return original(deadline)
        with patch.object(app,'old_snapshot',side_effect=snapshot):result=self.run_quiet()
        self.assertEqual(result['error'],'FROZEN_PARTIAL_ENTRY_MISSING')
        self.assertTrue(result['finalization_errors'])
    def test_second_run_does_not_overwrite(self):
        self.setup_run();self.run_quiet();before=(app.DOC/'transfer_manifest.json').read_bytes()
        with self.assertRaises(rio.IdentityError):self.run_quiet()
        self.assertEqual(before,(app.DOC/'transfer_manifest.json').read_bytes())
    def test_final_storage_receipt_failure_cannot_return_pass(self):
        self.setup_run();original=app.save
        def failing(path,value):
            if path.name=='storage_audit.json':raise OSError('synthetic receipt failure')
            return original(path,value)
        with patch.object(app,'save',side_effect=failing):result=self.run_quiet()
        self.assertEqual(result['result'],'ASSET_RECOVERY_BLOCKED_IDENTITY')
        self.assertTrue(json.loads((app.DOC/'transfer_manifest.json').read_text())['finalization_errors'])


if __name__=='__main__':unittest.main()
