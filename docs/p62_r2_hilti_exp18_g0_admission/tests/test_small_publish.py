import hashlib
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import acquire_assets as fixed
from publish_small import publish


class PublicationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='p62_publish_test_',dir='/home/jian/livox_ws')
        self.root=Path(self.temp.name);self.source=self.root/'source';self.source.write_bytes(b'abc')
        self.target=self.root/'assets/value';self.digest=hashlib.sha256(b'abc').hexdigest()
        self.root_patch=patch.object(fixed,'ROOT',self.root);self.root_patch.start()

    def tearDown(self):
        self.root_patch.stop();self.temp.cleanup()

    def test_staging_then_verified_publish(self):
        publish(self.source,self.target,3,self.digest)
        self.assertEqual(self.target.read_bytes(),b'abc')
        self.assertEqual((self.root/'staging/value.cached.part').stat().st_ino,self.target.stat().st_ino)

    def test_symlink_source_rejected(self):
        alias=self.root/'alias';alias.symlink_to(self.source)
        with self.assertRaisesRegex(RuntimeError,'NON_INTERNAL_SOURCE'):
            publish(alias,self.target,3,self.digest)
        self.assertFalse(self.target.exists())

    def test_failed_postcopy_verification_never_publishes(self):
        original=fixed.verify
        def checked(path,size,digest):
            if path==self.source:return original(path,size,digest)
            raise RuntimeError('POSTCOPY_MISMATCH')
        with patch.object(fixed,'verify',side_effect=checked):
            with self.assertRaisesRegex(RuntimeError,'POSTCOPY_MISMATCH'):
                publish(self.source,self.target,3,self.digest)
        self.assertFalse(self.target.exists())
        self.assertTrue((self.root/'staging/value.cached.part').exists())


if __name__=='__main__':unittest.main()
