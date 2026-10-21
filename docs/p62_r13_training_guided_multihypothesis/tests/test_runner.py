import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock,patch

source=Path(__file__).resolve().parents[1]/'execute.py'
spec=importlib.util.spec_from_file_location('r13_runner',source);runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)

class RunnerTests(unittest.TestCase):
    def invoke(self,child=None,error=None):
        with tempfile.TemporaryDirectory(prefix='r13_mock_only_') as d:
            with patch.object(runner.subprocess,'Popen',return_value=child,side_effect=error) as constructor:
                result=runner.invoke_once(['NO_REAL_ALGORITHM'],Path(d)/'log',{},timeout=1)
                self.assertEqual(constructor.call_count,1);return result
    def test_success(self):
        p=Mock();p.wait.return_value=0;r=self.invoke(p);self.assertEqual(r['returncode'],0);self.assertEqual(r['started_processes'],1);p.kill.assert_not_called()
    def test_nonzero(self):
        p=Mock();p.wait.return_value=7;self.assertEqual(self.invoke(p)['returncode'],7)
    def test_timeout_reaped(self):
        p=Mock();p.wait.side_effect=[subprocess.TimeoutExpired('MOCK',1),-9];r=self.invoke(p)
        self.assertEqual(r['returncode'],-9);self.assertTrue(r['timed_out']);p.kill.assert_called_once()
    def test_launch_failure(self):
        r=self.invoke(error=OSError('MOCK'));self.assertEqual(r['started_processes'],0);self.assertIsNone(r['returncode'])
    def test_independent_archive_errors(self):
        errors=[];result=runner.attempt('MOCK_DISK_FULL',lambda:(_ for _ in ()).throw(OSError('mock')),errors)
        self.assertIsNone(result);self.assertEqual(len(errors),1)
        self.assertEqual(runner.attempt('NEXT_STAGE',lambda:3,errors),3)

if __name__=='__main__':unittest.main()
