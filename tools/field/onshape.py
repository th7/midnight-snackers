#!/usr/bin/env python3
"""Talking to Onshape: signing a request, and refusing to pretend when the keys are missing.

Onshape signs a request with HMAC-SHA256 over the method, a nonce, the date, the content type,
the path and the query, lowercased and newline-separated, keyed by the secret. The access key
rides in the header in the clear to say who signed; only the secret is ever hashed, and neither
is ever written down.

Most of what the field document holds is public and needs no keys at all -- its metadata, its
element listing, its BOM and its blobs (the panel graphic, the April Tag images). Only geometry
is authenticated.

There are two ways a request here comes to be signed, and the difference is where the secret
lives rather than what reaches Onshape.

Under my-agent, an egress proxy signs `cad.onshape.com` on the way out. The container holds no
keys and needs none, and because the signature covers the path and query of the request the
proxy finally sends, a signature computed in here would not match: so the request goes bare, and
Authorization, Date and On-Nonce are the proxy's to set. This module will not set them when it
has no keys of its own.

Away from the proxy -- a teammate's laptop -- the key pair is in the environment and this signs
for itself. Keys come from the environment, never from a file in the repository:

    export ONSHAPE_ACCESS_KEY=...   # from dev-portal.onshape.com
    export ONSHAPE_SECRET_KEY=...

Either way:

    python3 onshape.py --check      # one call, to see that Onshape answers it

A client is built whichever is true, because which one is in force cannot be told from in here:
an empty environment means the proxy is signing, or that nothing is. Onshape settles it, and a
refusal says both things it could be rather than sending someone hunting for the wrong one.

No dependencies beyond Python 3.
"""
import argparse
import base64
import hashlib
import hmac
import secrets
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone

# The one call that reaches the network, named here so a test can stand in for Onshape.
urlopen = urllib.request.urlopen

BASE = 'https://cad.onshape.com'
# FIRST's "BIOBUZZ Playing Field", which the field model is built from. No version is pinned: we
# take whatever the document holds when we refresh the assets.
FIELD_DOCUMENT = 'a355e772e3d24813de7852ee'
FIELD_WORKSPACE = 'f106353168f1f92100b81259'
FIELD_ASSEMBLY = '95d1e1e442b4138cccaf2d73'
ACCESS_KEY_VARIABLE = 'ONSHAPE_ACCESS_KEY'
SECRET_KEY_VARIABLE = 'ONSHAPE_SECRET_KEY'
NONCE_LENGTH = 25
ALPHANUMERIC = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'


class NoCredentials(Exception):
    """Raised when a call that Onshape authenticates is made without a key pair. It names the
    variables that are missing and never the values of the ones that are set."""


def nonce():
    """A fresh nonce. Onshape rejects a repeat, which is what stops a captured request being
    replayed, so this must never be derived from anything a caller could repeat."""
    return ''.join(secrets.choice(ALPHANUMERIC) for _ in range(NONCE_LENGTH))


# `headers` takes a nonce of its own, which shadows the name; this is how it reaches the function.
fresh_nonce = nonce


def now():
    """The date in the form Onshape signs and checks, which is RFC 7231's, always in GMT."""
    return datetime.now(timezone.utc).strftime('%a, %d %b %Y %H:%M:%S GMT')


def split_url(url):
    """'/api/...?a=b' -> ('/api/...', 'a=b'). The signature covers the two separately, so a caller
    that folds the query into the path signs a string the server will not reproduce."""
    path, _, query = url.partition('?')
    return path, query


def signature(method, path, query, nonce, date, content_type, secret):
    """The signature over one call. Onshape lowercases the whole string before hashing it, so the
    case a caller spells the method or the path in does not matter -- but the order and the
    trailing newline do."""
    signed = '\n'.join([method, nonce, date, content_type, path, query, '']).lower()
    digest = hmac.new(secret.encode('utf-8'), signed.encode('utf-8'), hashlib.sha256).digest()
    return base64.b64encode(digest).decode('ascii')


def authorization(access, signature, nonce):
    """The Authorization header: who signed, how, and the signature itself."""
    return 'On ' + access + ':HmacSHA256:' + signature


def headers(method, url_path, query, access=None, secret=None, nonce=None, date=None,
            content_type='application/json'):
    """The headers for one call. Without a key pair the call goes out unsigned, which is all the
    public parts of the document need; with one, the nonce and date in the headers are the ones
    the signature covers, or the server recomputes a different hash."""
    sent = {'Accept': 'application/json', 'Content-Type': content_type}
    if access is None or secret is None:
        return sent
    nonce = nonce or fresh_nonce()
    date = date or now()
    sent['On-Nonce'] = nonce
    sent['Date'] = date
    sent['Authorization'] = authorization(
        access, signature(method, url_path, query, nonce, date, content_type, secret), nonce)
    return sent


def credentials():
    """The key pair from the environment, or a refusal naming what is missing. A variable set to
    blank counts as missing: an empty secret would sign every request to the same wrong value and
    fail far from here, with a 401 that looks like a permissions problem."""
    import os
    values = {name: (os.environ.get(name) or '').strip()
              for name in (ACCESS_KEY_VARIABLE, SECRET_KEY_VARIABLE)}
    missing = sorted(name for name, value in values.items() if not value)
    if missing:
        raise NoCredentials(
            'Onshape geometry needs a key pair and ' + ' and '.join(missing)
            + (' are not set. ' if len(missing) > 1 else ' is not set. ')
            + 'Make one at dev-portal.onshape.com and export it; never commit it. '
            '(The public parts of the document -- metadata, BOM, blobs -- need no keys.)')
    return values[ACCESS_KEY_VARIABLE], values[SECRET_KEY_VARIABLE]


def optional_credentials():
    """The key pair if this machine has one, or (None, None) if it does not. Half a pair is no
    pair: signing with a blank secret would fail at Onshape as a puzzling 401 rather than here."""
    try:
        return credentials()
    except NoCredentials:
        return None, None


class Client:
    """A caller for one Onshape server. Built without a key pair it can still reach everything
    public; `authenticated()` is how a caller that needs geometry insists on one."""

    def __init__(self, access=None, secret=None, base=BASE):
        self._access = access
        self._secret = secret
        self.base = base

    @classmethod
    def authenticated(cls, base=BASE):
        """A client with keys from the environment, or NoCredentials. For a caller that must sign
        for itself; most callers want `configured`."""
        access, secret = credentials()
        return cls(access=access, secret=secret, base=base)

    @classmethod
    def configured(cls, base=BASE):
        """However this machine reaches Onshape.

        With a key pair in the environment the client signs for itself, which is how a teammate
        works away from my-agent. Without one it sends the request bare, because an egress proxy
        may be signing on the way out -- and where it is, this container holds no keys and needs
        none. Refusing to build a client here would stop a run that would have worked; whether
        anything authenticated the request is Onshape's to say, and `get` reports its answer.
        """
        access, secret = optional_credentials()
        return cls(access=access, secret=secret, base=base)

    @property
    def signed(self):
        return self._access is not None and self._secret is not None

    def __repr__(self):
        """Reaches logs and tracebacks, so it says whether there are keys and never what they are."""
        return '<Onshape ' + self.base + (' signed>' if self.signed else ' anonymous>')

    def get(self, url, accept='application/json'):
        """One GET, as bytes. `url` is the path with its query, from '/api/...'."""
        path, query = split_url(url)
        sent = headers('GET', path, query, self._access, self._secret)
        sent['Accept'] = accept
        request = urllib.request.Request(self.base + url, headers=sent, method='GET')
        try:
            with urlopen(request) as response:
                return response.read()
        except urllib.error.HTTPError as error:
            body = error.read().decode('utf-8', 'replace')[:400]
            if error.code in (401, 403):
                raise NoCredentials(self._refusal(path, error.code, body)) from None
            raise RuntimeError('Onshape ' + str(error.code) + ' for ' + path + ': ' + body) from None

    def _refusal(self, path, code, body):
        """What to do about a 401, which depends on who was supposed to have signed. Sending
        someone hunting for environment variables when the answer is that the proxy's onshape
        service is switched off would cost an afternoon."""
        if self.signed:
            return ('Onshape refused the signed request for ' + path + ' (' + str(code)
                    + '): the key pair was refused -- revoked, deleted, or scoped away from this '
                    'document. Make a new one and set ' + ACCESS_KEY_VARIABLE + ' and '
                    + SECRET_KEY_VARIABLE + '. ' + body)
        return ('Onshape refused an unsigned request for ' + path + ' (' + str(code)
                + ') and nothing authenticated it. Either an egress proxy is meant to be signing '
                'cad.onshape.com on the way out and its onshape service is not switched on, or '
                'there is no proxy here and this machine needs its own key pair in '
                + ACCESS_KEY_VARIABLE + ' and ' + SECRET_KEY_VARIABLE + '. ' + body)


def check(document, workspace):
    """One authenticated call, to see that Onshape answers it. Whoever signed -- the proxy, or
    this module with keys from the environment -- only the server can say the signature was
    right: the unit tests pin the algorithm so it cannot drift, and this is what proves it."""
    client = Client.configured()
    url = '/api/v10/documents/d/' + document + '/w/' + workspace + '/currentmicroversion'
    body = client.get(url).decode('utf-8', 'replace')
    print('Onshape accepted the signature.')
    print(body[:200])


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', action='store_true',
                        help='make one signed call and report whether Onshape accepted it')
    parser.add_argument('--document', default=FIELD_DOCUMENT,
                        help="the field document (default: FIRST's BIOBUZZ Playing Field)")
    parser.add_argument('--workspace', default=FIELD_WORKSPACE)
    args = parser.parse_args(argv)
    if not args.check:
        parser.error('nothing to do; --check makes one signed call')
    try:
        check(args.document, args.workspace)
    except NoCredentials as refusal:
        print(refusal, file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
