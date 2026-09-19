import json
import os
import tempfile
import unittest

import assets

PNG = b'\x89PNG\r\n\x1a\n' + b'\x00' * 40
ELEMENTS = [
    {'name': 'am-5850 BIOBUZZ', 'id': 'aaa', 'elementType': 'ASSEMBLY'},
    {'name': 'GoalAprilTag_bluescoring.png', 'id': 'b1', 'elementType': 'BLOB'},
    {'name': 'GoalAprilTag_blueaudience.png', 'id': 'b2', 'elementType': 'BLOB'},
    {'name': 'GoalAprilTag_redscoring.png', 'id': 'b3', 'elementType': 'BLOB'},
    {'name': 'GoalAprilTag_redaudience.png', 'id': 'b4', 'elementType': 'BLOB'},
    {'name': 'BIOBUZZ_Panel_Resized.png', 'id': 'b5', 'elementType': 'BLOB'},
    {'name': 'am-5850 BIOBUZZ 8-10-26.STEP', 'id': 'b6', 'elementType': 'BLOB'},
]

class FakeClient:

    def __init__(self, elements=None, blobs=None):
        self.elements = ELEMENTS if elements is None else elements
        self.blobs = blobs if blobs is not None else {e['id']: PNG for e in ELEMENTS}
        self.asked = []

    def get(self, url, accept='application/json'):
        self.asked.append(url)
        if url.endswith('/elements'):
            return json.dumps(self.elements).encode('utf-8')
        element = url.rstrip('/').rsplit('/', 1)[-1]
        if element not in self.blobs:
            raise RuntimeError('Onshape 404 for ' + element)
        return self.blobs[element]

class Listing(unittest.TestCase):
    def test_only_blobs_are_offered_as_assets(self):
        found = assets.blobs(FakeClient(), 'doc', 'work')
        self.assertNotIn('am-5850 BIOBUZZ', found)
        self.assertIn('BIOBUZZ_Panel_Resized.png', found)
        self.assertEqual('b5', found['BIOBUZZ_Panel_Resized.png'])

    def test_the_step_blob_is_listed_but_not_a_texture(self):
        self.assertIn('am-5850 BIOBUZZ 8-10-26.STEP', assets.blobs(FakeClient(), 'doc', 'work'))
        self.assertNotIn('am-5850 BIOBUZZ 8-10-26.STEP', assets.TEXTURES)

class Fetching(unittest.TestCase):
    def test_every_texture_is_written_and_named_as_the_document_names_it(self):
        client = FakeClient()
        with tempfile.TemporaryDirectory() as into:
            written = assets.fetch(client, 'doc', 'work', into)
            self.assertEqual(sorted(assets.TEXTURES), sorted(os.path.basename(p) for p in written))
            for path in written:
                self.assertEqual(PNG, open(path, 'rb').read())

    def test_a_texture_the_document_no_longer_has_stops_the_run(self):
        thinner = [e for e in ELEMENTS if e['name'] != 'GoalAprilTag_redscoring.png']
        with tempfile.TemporaryDirectory() as into:
            with self.assertRaises(assets.Missing) as raised:
                assets.fetch(FakeClient(elements=thinner), 'doc', 'work', into)
            self.assertIn('GoalAprilTag_redscoring.png', str(raised.exception))
            self.assertEqual([], os.listdir(into), 'nothing is written when one is missing')

    def test_something_that_is_not_a_png_is_refused(self):
        client = FakeClient(blobs=dict({e['id']: PNG for e in ELEMENTS},
                                       b5=b'<!doctype html><html>no</html>'))
        with tempfile.TemporaryDirectory() as into:
            with self.assertRaises(assets.NotAnImage) as raised:
                assets.fetch(client, 'doc', 'work', into)
            self.assertIn('BIOBUZZ_Panel_Resized.png', str(raised.exception))
            self.assertEqual([], os.listdir(into))

    def test_an_empty_answer_is_refused(self):
        client = FakeClient(blobs=dict({e['id']: PNG for e in ELEMENTS}, b1=b''))
        with tempfile.TemporaryDirectory() as into:
            with self.assertRaises(assets.NotAnImage):
                assets.fetch(client, 'doc', 'work', into)

    def test_nothing_is_fetched_twice(self):
        client = FakeClient()
        with tempfile.TemporaryDirectory() as into:
            assets.fetch(client, 'doc', 'work', into)
        blob_calls = [u for u in client.asked if 'blobelements' in u]
        self.assertEqual(len(assets.TEXTURES), len(blob_calls))

class Names(unittest.TestCase):

    def test_a_name_that_climbs_out_of_the_destination_is_refused(self):
        for hostile in ('../escaped.png', 'a/../../escaped.png', '/etc/escaped.png'):
            with self.subTest(hostile):
                with self.assertRaises(assets.BadName):
                    assets.safe_name(hostile)

    def test_a_name_with_a_directory_in_it_is_refused(self):
        with self.assertRaises(assets.BadName):
            assets.safe_name('textures/panel.png')

    def test_an_ordinary_name_is_kept_exactly(self):
        self.assertEqual('GoalAprilTag_bluescoring.png',
                         assets.safe_name('GoalAprilTag_bluescoring.png'))

    def test_a_hostile_name_in_the_document_stops_the_run(self):
        crooked = [dict(e) for e in ELEMENTS]
        for element in crooked:
            if element['name'] == 'BIOBUZZ_Panel_Resized.png':
                element['name'] = '../BIOBUZZ_Panel_Resized.png'
        client = FakeClient(elements=crooked)
        with tempfile.TemporaryDirectory() as into:
            with self.assertRaises((assets.BadName, assets.Missing)):
                assets.fetch(client, 'doc', 'work', into)
            self.assertEqual([], os.listdir(into))

if __name__ == '__main__':
    unittest.main()
