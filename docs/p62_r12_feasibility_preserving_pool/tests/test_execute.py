import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

path=Path(__file__).resolve().parents[1]/'execute.py'
spec=importlib.util.spec_from_file_location('r12_execute',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class OneCallTests(unittest.TestCase):
    def invoke(self,result=None,error=None):
        with tempfile.TemporaryDirectory(prefix='r12_zero_algorithm_test_') as directory:
            with patch.object(module.subprocess,'run',return_value=result,side_effect=error) as mocked:
                record=module.invoke_once(['NO_REAL_ALGORITHM'],Path(directory)/'test.log',{})
                self.assertEqual(mocked.call_count,1)
                return record
    def test_success(self):
        r=self.invoke(subprocess.CompletedProcess([],0));self.assertEqual(r['returncode'],0);self.assertIsNone(r['error'])
    def test_nonzero_preserved(self):self.assertEqual(self.invoke(subprocess.CompletedProcess([],7))['returncode'],7)
    def test_timeout_no_retry(self):
        r=self.invoke(error=subprocess.TimeoutExpired('MOCK',60));self.assertIsNone(r['returncode']);self.assertIn('TimeoutExpired',r['error'])
    def test_start_failure_no_retry(self):self.assertIn('OSError',self.invoke(error=OSError('MOCK'))['error'])
    def test_historical_output_exclusion(self):
        self.assertNotEqual(module.DOC,module.R11)
        self.assertEqual(module.REPO.name,'p62_r12_feasibility_pool_ws')
        self.assertNotEqual(module.RUN,module.R11)

if __name__=='__main__':unittest.main()
