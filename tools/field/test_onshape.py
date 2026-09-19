import os
import unittest

import onshape

ACCESS = 'AAAAAAAAAAAAAAAAAAAAAAAA'
SECRET = 'BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'
NONCE = 'AbCdEfGhIjKlMnOpQrStUvWxY'
DATE = 'Mon, 18 Sep 2026 12:00:00 GMT'

def sign(**kwargs):
    call = dict(method='GET', path='/api/v10/documents/abc', query='', nonce=NONCE, date=DATE,
                content_type='application/json', secret=SECRET)
    call.update(kwargs)
    return onshape.signature(**call)

class Signing(unittest.TestCase):
    def test_a_known_call_signs_to_a_known_string(self):
        self.assertEqual(
            'RAElJvpy1mwpaT9YvdNVKGITbAapDFv0aOfvrF6R7XM=',
            sign())

    def test_every_part_of_the_call_is_signed(self):
        for name, changed in [('method', 'POST'), ('path', '/api/v10/documents/xyz'),
                              ('query', 'foo=bar'), ('nonce', 'ZzZzZzZzZzZzZzZzZzZzZzZzZ'),
                              ('date', 'Tue, 19 Sep 2026 12:00:00 GMT'),
                              ('content_type', 'text/plain'), ('secret', 'C' * 44)]:
            with self.subTest(name):
                self.assertNotEqual(sign(), sign(**{name: changed}), name + ' is not signed')

    def test_the_case_of_the_call_does_not_change_the_signature(self):
        self.assertEqual(sign(), sign(method='get'))
        self.assertEqual(sign(), sign(path='/API/V10/Documents/ABC'))

    def test_the_access_key_is_not_signed_but_names_the_signer(self):
        header = onshape.authorization(ACCESS, sign(), NONCE)
        self.assertTrue(header.startswith('On ' + ACCESS + ':HmacSHA256:'), header)
        self.assertIn(sign(), header)

class Headers(unittest.TestCase):
    def test_a_signed_request_carries_what_the_signature_covers(self):
        headers = onshape.headers('GET', '/api/v10/documents/abc', '', ACCESS, SECRET,
                                  nonce=NONCE, date=DATE)
        self.assertEqual(NONCE, headers['On-Nonce'])
        self.assertEqual(DATE, headers['Date'])
        self.assertIn(sign(), headers['Authorization'])

    def test_each_request_gets_a_fresh_nonce(self):
        nonces = {onshape.headers('GET', '/api/v10/documents/abc', '', ACCESS, SECRET)['On-Nonce']
                  for _ in range(50)}
        self.assertEqual(50, len(nonces))

    def test_the_nonce_is_long_enough_to_be_unguessable(self):
        nonce = onshape.nonce()
        self.assertGreaterEqual(len(nonce), 25)
        self.assertTrue(nonce.isalnum(), nonce)

class MissingKeys(unittest.TestCase):

    def setUp(self):
        self.saved = {k: os.environ.pop(k, None)
                      for k in ('ONSHAPE_ACCESS_KEY', 'ONSHAPE_SECRET_KEY')}

    def tearDown(self):
        for k, v in self.saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    def test_credentials_without_keys_raise_naming_both_variables(self):
        with self.assertRaises(onshape.NoCredentials) as raised:
            onshape.credentials()
        self.assertIn('ONSHAPE_ACCESS_KEY', str(raised.exception))
        self.assertIn('ONSHAPE_SECRET_KEY', str(raised.exception))

    def test_half_a_key_pair_is_still_missing(self):
        os.environ['ONSHAPE_ACCESS_KEY'] = ACCESS
        with self.assertRaises(onshape.NoCredentials) as raised:
            onshape.credentials()
        self.assertIn('ONSHAPE_SECRET_KEY', str(raised.exception))

    def test_a_blank_key_is_missing_rather_than_a_key(self):
        os.environ['ONSHAPE_ACCESS_KEY'] = ACCESS
        os.environ['ONSHAPE_SECRET_KEY'] = '   '
        with self.assertRaises(onshape.NoCredentials):
            onshape.credentials()

    def test_an_anonymous_request_needs_no_keys_and_sends_no_authorization(self):
        self.assertNotIn('Authorization', onshape.headers('GET', '/api/v10/blobelements/d/a/w/b/e/c', ''))

    def test_a_client_without_keys_is_built_rather_than_refused(self):
        client = onshape.Client.configured()
        self.assertFalse(client.signed)

    def test_a_client_signs_for_itself_when_the_keys_are_in_the_environment(self):
        os.environ['ONSHAPE_ACCESS_KEY'] = ACCESS
        os.environ['ONSHAPE_SECRET_KEY'] = SECRET
        self.assertTrue(onshape.Client.configured().signed)

class ProxySigns(unittest.TestCase):

    def test_an_unsigned_client_sets_none_of_the_three_headers_the_proxy_owns(self):
        sent = onshape.headers('GET', '/api/v10/assemblies/d/a/w/b/e/c/gltf', '')
        for header in ('Authorization', 'Date', 'On-Nonce'):
            self.assertNotIn(header, sent, header + ' is the proxy\'s to set')

    def test_a_refusal_of_an_unsigned_request_names_both_ways_to_be_authenticated(self):
        client = onshape.Client()
        with Refusing(401):
            with self.assertRaises(onshape.NoCredentials) as raised:
                client.get('/api/v10/assemblies/d/a/w/b/e/c/gltf')
        message = str(raised.exception)
        self.assertIn('ONSHAPE_ACCESS_KEY', message)
        self.assertIn('proxy', message)

    def test_a_refusal_of_a_signed_request_says_the_keys_were_refused(self):
        client = onshape.Client(access=ACCESS, secret=SECRET)
        with Refusing(403):
            with self.assertRaises(onshape.NoCredentials) as raised:
                client.get('/api/v10/assemblies/d/a/w/b/e/c/gltf')
        self.assertIn('refused', str(raised.exception).lower())
        self.assertNotIn(SECRET, str(raised.exception))

    def test_a_failure_that_is_not_about_credentials_is_not_reported_as_one(self):
        client = onshape.Client()
        with Refusing(500):
            with self.assertRaises(RuntimeError) as raised:
                client.get('/api/v10/assemblies/d/a/w/b/e/c/gltf')
        self.assertNotIsInstance(raised.exception, onshape.NoCredentials)

class Refusing:

    def __init__(self, status):
        self.status = status
        self.saved = None

    def __enter__(self):
        import io
        import urllib.error
        status = self.status
        self.saved = onshape.urlopen

        def refuse(request, *args, **kwargs):
            raise urllib.error.HTTPError(
                request.full_url, status, 'no', {},
                io.BytesIO(b'{"message": "Unauthenticated API request"}'))

        onshape.urlopen = refuse
        return self

    def __exit__(self, *exception):
        onshape.urlopen = self.saved
        return False

class Secrecy(unittest.TestCase):
    def test_the_secret_does_not_appear_in_the_missing_credentials_message(self):
        os.environ['ONSHAPE_ACCESS_KEY'] = ACCESS
        os.environ['ONSHAPE_SECRET_KEY'] = ''
        try:
            with self.assertRaises(onshape.NoCredentials) as raised:
                onshape.credentials()
            self.assertNotIn(ACCESS, str(raised.exception))
        finally:
            del os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY']

    def test_a_client_does_not_print_its_keys(self):
        client = onshape.Client(access=ACCESS, secret=SECRET)
        self.assertNotIn(ACCESS, repr(client))
        self.assertNotIn(SECRET, repr(client))

class Urls(unittest.TestCase):
    def test_a_path_and_its_query_are_kept_apart(self):
        path, query = onshape.split_url('/api/v10/documents/d/a/w/b/elements?elementType=BLOB')
        self.assertEqual('/api/v10/documents/d/a/w/b/elements', path)
        self.assertEqual('elementType=BLOB', query)

    def test_a_url_without_a_query_has_an_empty_one(self):
        path, query = onshape.split_url('/api/v10/documents/abc')
        self.assertEqual('/api/v10/documents/abc', path)
        self.assertEqual('', query)

if __name__ == '__main__':
    unittest.main()
