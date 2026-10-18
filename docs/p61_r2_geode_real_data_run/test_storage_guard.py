import unittest
from storage_transfer import ERROR, evidence_path


class StorageGuardTest(unittest.TestCase):
    def test_resume_keeps_previous_evidence_separate(self):
        self.assertNotEqual(evidence_path('copy'), evidence_path('copy', 'resume1'))
        for attempt in ('../copy', '', 'a/b'):
            with self.assertRaises(ValueError): evidence_path('copy', attempt)

    def test_associated_storage_faults_stop(self):
        for line in ('usb 4-2: reset SuperSpeed USB device', 'usb 4-2: USB disconnect',
                     'blk_update_request: I/O error, dev sda, sector 100',
                     'sd 0:0:0:0: [sda] tag#3 FAILED Result: hostbyte=DID_ERROR',
                     'scsi host0: uas_eh_device_reset_handler start', 'exfat-fs (sda2): failed write',
                     'lost async page write'):
            self.assertIsNotNone(ERROR.search(line), line)

    def test_unrelated_usb_or_normal_state_is_not_disk_failure(self):
        for line in ('usb 1-3: USB disconnect', 'PID 1234 D wait_on_page_bit',
                     'sda: detected capacity change', 'copy complete'):
            self.assertIsNone(ERROR.search(line), line)


if __name__ == '__main__': unittest.main()
