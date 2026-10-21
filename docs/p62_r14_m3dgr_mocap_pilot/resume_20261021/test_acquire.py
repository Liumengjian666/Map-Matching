import importlib.util
import io
from pathlib import Path
import tempfile
import time
import hashlib
import unittest
from unittest.mock import patch

path=Path(__file__).with_name('acquire.py')
spec=importlib.util.spec_from_file_location('resume_acquire',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class BoundedAccessTests(unittest.TestCase):
    def test_total_budget_and_reserve(self):
        self.assertTrue(module.capacity(module.RESERVE+sum(module.LIMITS.values())))
        self.assertFalse(module.capacity(module.RESERVE+sum(module.LIMITS.values())-1))
    def test_no_write_below_reserve(self):
        self.assertFalse(module.write_allowed(module.RESERVE,0,1,module.LIMITS['bag']))
    def test_near_category_limit_refused(self):
        self.assertFalse(module.write_allowed(10**12,module.LIMITS['bag']-module.NEAR-1,1,module.LIMITS['bag']))
    def test_html_not_a_bag(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);result=module.receive(io.BytesIO(b'<html>login required</html>'),root/'asset.bag',root/'sample',free=lambda:10**12)
            self.assertEqual(result['status'],'ASSET_ACCESS_REQUIRES_USER_ACTION')
            self.assertFalse((root/'asset.bag').exists());self.assertFalse((root/'asset.bag.part').exists())
    def test_landing_sample_hard_cap(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);result=module.receive(io.BytesIO(b'<html>'+b'x'*200000),root/'asset.bag',root/'sample',free=lambda:10**12)
            self.assertEqual(result['sample_bytes'],65536)
    def test_wrong_digest_never_published(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);result=module.receive(io.BytesIO(module.MAGIC+b'fake'),root/'asset.bag',root/'sample',free=lambda:10**12)
            self.assertEqual(result['status'],'ASSET_IDENTITY_MISMATCH');self.assertFalse((root/'asset.bag').exists())
    def test_capacity_failure_stops_before_writing(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            with self.assertRaises(RuntimeError):module.receive(io.BytesIO(module.MAGIC+b'fake'),root/'asset.bag',root/'sample',free=lambda:module.RESERVE)
            self.assertFalse((root/'asset.bag').exists());self.assertEqual((root/'asset.bag.part').stat().st_size,0)
    def test_existing_partial_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);partial=root/'asset.bag.part';partial.write_bytes(b'existing')
            with self.assertRaises(FileExistsError):module.receive(io.BytesIO(module.MAGIC),root/'asset.bag',root/'sample',free=lambda:10**12)
            self.assertEqual(partial.read_bytes(),b'existing')
    def test_external_redirect_rejected(self):
        req=module.urllib.request.Request(module.URL)
        with self.assertRaises(RuntimeError):module.OfficialRedirects().redirect_request(req,None,302,'',{},'https://example.com/login')
    def test_self_redirect_not_requested_twice(self):
        req=module.urllib.request.Request(module.URL)
        with self.assertRaises(module.RedirectRequiresUserAction):module.OfficialRedirects().redirect_request(req,None,302,'',{},module.URL+'?again=1')
    def test_default_https_port_same_page(self):
        req=module.urllib.request.Request(module.URL)
        with self.assertRaises(module.RedirectRequiresUserAction):
            module.OfficialRedirects().redirect_request(req,None,302,'',{},'https://www.alipan.com:443/s/aVQ24Hm4Xcm')
    def test_https_downgrade_rejected(self):
        req=module.urllib.request.Request(module.URL)
        with self.assertRaises(module.RedirectRequiresUserAction):
            module.OfficialRedirects().redirect_request(req,None,302,'',{},'http://www.alipan.com/s/other')
    def test_cyclic_redirect_rejected(self):
        handler=module.OfficialRedirects();req=module.urllib.request.Request(module.URL)
        next_req=handler.redirect_request(req,None,302,'',{},'https://www.aliyundrive.com/s/test')
        with self.assertRaises(module.RedirectRequiresUserAction):handler.redirect_request(next_req,None,302,'',{},module.URL)
    def test_global_deadline_covers_slow_html_first_read(self):
        class SlowPage:
            def read(self,size):time.sleep(.2);return b'<html>'
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            with self.assertRaises(TimeoutError):
                with module.request_deadline(.02):module.receive(SlowPage(),root/'asset.bag',root/'sample',free=lambda:10**12)
            self.assertFalse((root/'asset.bag').exists())
    def test_synthetic_matching_digest_publication(self):
        data=module.MAGIC+b'synthetic only: not a valid parsed bag'
        with tempfile.TemporaryDirectory() as temp,patch.object(module,'EXPECTED',hashlib.sha256(data).hexdigest()):
            root=Path(temp);result=module.receive(io.BytesIO(data),root/'asset.bag',root/'sample',free=lambda:10**12)
            self.assertEqual(result['status'],'BAG_SHA_AND_MAGIC_VERIFIED')
            self.assertEqual((root/'asset.bag').read_bytes(),data)

if __name__=='__main__':unittest.main()
