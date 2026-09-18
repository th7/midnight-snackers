"""The Onshape client: how a request is signed, and what happens when the keys are missing.

These tests never touch the network. They pin the shape of the signature so it cannot drift
silently, and they pin the two refusals that matter: a call that needs keys and has none must
say so, and a secret must never reach a message anyone might log.

What they cannot prove is that Onshape accepts the signature -- only Onshape can say that, and
only with real keys. `onshape.py --check` makes that one call.
"""
import os
import unittest

import onshape

# A key pair that never existed, fixed so the signature below is reproducible.
ACCESS = 'AAAAAAAAAAAAAAAAAAAAAAAA'
SECRET = 'BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'
NONCE = 'AbCdEfGhIjKlMnOpQrStUvWxY'
DATE = 'Mon, 18 Sep 2026 12:00:00 GMT'


def sign(**kwargs):
    """The signing inputs, with anything not named taking a fixed default."""
    call = dict(method='GET', path='/api/v10/documents/abc', query='', nonce=NONCE, date=DATE,
                content_type='application/json', secret=SECRET)
    call.update(kwargs)
    return onshape.signature(**call)


class Signing(unittest.TestCase):
    def test_a_known_call_signs_to_a_known_string(self):
        """The algorithm itself, pinned. This says the implementation has not drifted; it does not
        say Onshape agrees, which is what --check is for.

        The expected value is derived from Onshape's documented scheme rather than read back out
        of this module, so that a change here that still hashes *something* is caught. It is
        HMAC-SHA256 over, lowercased, each followed by a newline: the method, the nonce, the date,
        the content type, the path and the query --

            'get\\nabcdefghijklmnopqrstuvwxy\\nmon, 18 sep 2026 12:00:00 gmt\\n'
            'application/json\\n/api/v10/documents/abc\\n\\n'

        keyed by SECRET, base64-encoded."""
        self.assertEqual(
            'RAElJvpy1mwpaT9YvdNVKGITbAapDFv0aOfvrF6R7XM=',
            sign())

    def test_every_part_of_the_call_is_signed(self):
        """Each component must change the signature, or it is not in the string being hashed and an
        attacker could vary it freely."""
        for name, changed in [('method', 'POST'), ('path', '/api/v10/documents/xyz'),
                              ('query', 'foo=bar'), ('nonce', 'ZzZzZzZzZzZzZzZzZzZzZzZzZ'),
                              ('date', 'Tue, 19 Sep 2026 12:00:00 GMT'),
                              ('content_type', 'text/plain'), ('secret', 'C' * 44)]:
            with self.subTest(name):
                self.assertNotEqual(sign(), sign(**{name: changed}), name + ' is not signed')

    def test_the_case_of_the_call_does_not_change_the_signature(self):
        """Onshape lowercases the string it signs, so a caller that spells the method or the path
        differently must still produce the same signature."""
        self.assertEqual(sign(), sign(method='get'))
        self.assertEqual(sign(), sign(path='/API/V10/Documents/ABC'))

    def test_the_access_key_is_not_signed_but_names_the_signer(self):
        """The header carries the access key in the clear; only the secret is hashed."""
        header = onshape.authorization(ACCESS, sign(), NONCE)
        self.assertTrue(header.startswith('On ' + ACCESS + ':HmacSHA256:'), header)
        self.assertIn(sign(), header)


class Headers(unittest.TestCase):
    def test_a_signed_request_carries_what_the_signature_covers(self):
        """The nonce and date in the headers must be the ones that were signed, or the server
        recomputes a different hash."""
        headers = onshape.headers('GET', '/api/v10/documents/abc', '', ACCESS, SECRET,
                                  nonce=NONCE, date=DATE)
        self.assertEqual(NONCE, headers['On-Nonce'])
        self.assertEqual(DATE, headers['Date'])
        self.assertIn(sign(), headers['Authorization'])

    def test_each_request_gets_a_fresh_nonce(self):
        """A repeated nonce is a replay; Onshape rejects it, and so should we never send one."""
        nonces = {onshape.headers('GET', '/api/v10/documents/abc', '', ACCESS, SECRET)['On-Nonce']
                  for _ in range(50)}
        self.assertEqual(50, len(nonces))

    def test_the_nonce_is_long_enough_to_be_unguessable(self):
        nonce = onshape.nonce()
        self.assertGreaterEqual(len(nonce), 25)
        self.assertTrue(nonce.isalnum(), nonce)


class MissingKeys(unittest.TestCase):
    """A run without keys must stop and say so. The failure this guards against is a pipeline that
    quietly regenerates nothing and leaves yesterday's assets in place, looking like it worked."""

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
        """The document's metadata, its blobs and its BOM are public; only geometry needs keys.
        Fetching a texture must not demand a key pair nobody needs."""
        self.assertNotIn('Authorization', onshape.headers('GET', '/api/v10/blobelements/d/a/w/b/e/c', ''))


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
        """repr() reaches logs and tracebacks; the keys must not ride along."""
        client = onshape.Client(access=ACCESS, secret=SECRET)
        self.assertNotIn(ACCESS, repr(client))
        self.assertNotIn(SECRET, repr(client))


class Urls(unittest.TestCase):
    def test_a_path_and_its_query_are_kept_apart(self):
        """The signature covers them separately, so a client that folds the query into the path
        signs a string the server will not reproduce."""
        path, query = onshape.split_url('/api/v10/documents/d/a/w/b/elements?elementType=BLOB')
        self.assertEqual('/api/v10/documents/d/a/w/b/elements', path)
        self.assertEqual('elementType=BLOB', query)

    def test_a_url_without_a_query_has_an_empty_one(self):
        path, query = onshape.split_url('/api/v10/documents/abc')
        self.assertEqual('/api/v10/documents/abc', path)
        self.assertEqual('', query)


if __name__ == '__main__':
    unittest.main()
