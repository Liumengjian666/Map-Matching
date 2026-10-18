import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from list_drive_metadata import listing
from probe_bag_access import FILE_ID, parse_confirmation
from snapshot_metadata import main as snapshot


class MetadataTests(unittest.TestCase):
    def test_public_listing_hex_decoding(self):
        data = [[['abc', ['parent'], 'Shield_tunnel1_gamma.bag', 'application/octet-stream']]]
        encoded = json.dumps(data).replace('[', r'\x5b').replace(']', r'\x5d').replace('"', r'\x22')
        rows = listing("window['_DRIVE_ivd'] = '" + encoded + "';")
        self.assertEqual(rows[0]['id'], 'abc')
        self.assertEqual(rows[0]['name'], 'Shield_tunnel1_gamma.bag')

    def test_missing_or_unrecognized_listing_rejected(self):
        for text in ('<html>Login required</html>', "window['_DRIVE_ivd'] = '[[[1,2,3]]]';"):
            with self.assertRaises(ValueError): listing(text)

    def test_confirmation_must_be_same_official_file(self):
        page = ('<form action="https://drive.usercontent.google.com/download">'
                '<input name="id" value="' + FILE_ID + '"><input name="export" value="download">'
                '<input name="confirm" value="t"></form>')
        self.assertEqual(parse_confirmation(page).values['id'], FILE_ID)
        for bad in (page.replace(FILE_ID, 'different_file'), page.replace('drive.usercontent.google.com', 'example.com'),
                    '<html>quota exceeded</html>'):
            with self.assertRaises(ValueError): parse_confirmation(bad)

    def test_metadata_does_not_overwrite(self):
        with tempfile.TemporaryDirectory(dir='/home/jian') as directory:
            target = Path(directory) / 'existing.html'; target.write_bytes(b'frozen')
            with patch('snapshot_metadata.requests.get') as request:
                with self.assertRaises(ValueError): snapshot('https://drive.google.com/test', str(target))
                request.assert_not_called()
            self.assertEqual(target.read_bytes(), b'frozen')

    def test_external_destination_and_nonofficial_source_rejected(self):
        with patch('snapshot_metadata.requests.get') as request:
            for url, destination in (('https://example.com/a', '/home/jian/new'),
                                     ('https://drive.google.com/a', '/media/forbidden/test')):
                with self.assertRaises(ValueError): snapshot(url, destination)
            request.assert_not_called()

    def test_http_200_quota_is_not_sensor_input(self):
        doc = Path(__file__).resolve().parent
        result = json.loads((doc / 'results.json').read_text())
        access = json.loads((doc / 'access_evidence/access_receipt.json').read_text())
        body = (doc / 'access_evidence/response_body.bin').read_bytes()
        self.assertEqual(access['status'], 200)
        self.assertTrue(access['quota_message'])
        self.assertFalse(body.startswith(b'#ROSBAG V2.0\n'))
        self.assertFalse(result['complete_bag_obtained'])
        self.assertIsNone(result['complete_bag_sha256'])
        self.assertEqual(result['ndt_align_calls'], 0)

    def test_map_and_calibration_not_silently_promoted(self):
        doc = Path(__file__).resolve().parent
        result = json.loads((doc / 'results.json').read_text())
        manifest = json.loads((doc / 'source_manifest.json').read_text())
        self.assertEqual([r['name'] for r in manifest['public_folder_listings']['map_listing']], ['stairs.las'])
        self.assertFalse(result['independent_tunnel_map_found'])
        self.assertFalse(result['extrinsic_approved'])
        self.assertFalse(result['smoke_completed'])


if __name__ == '__main__':
    unittest.main()
