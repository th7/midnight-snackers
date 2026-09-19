#!/usr/bin/env python3
import argparse
import base64
import hashlib
import hmac
import secrets
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone

urlopen = urllib.request.urlopen

BASE = 'https://cad.onshape.com'
FIELD_DOCUMENT = 'a355e772e3d24813de7852ee'
FIELD_WORKSPACE = 'f106353168f1f92100b81259'
FIELD_ASSEMBLY = '95d1e1e442b4138cccaf2d73'
ACCESS_KEY_VARIABLE = 'ONSHAPE_ACCESS_KEY'
SECRET_KEY_VARIABLE = 'ONSHAPE_SECRET_KEY'
NONCE_LENGTH = 25
ALPHANUMERIC = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'

class NoCredentials(Exception):
    pass

def nonce():
    return ''.join(secrets.choice(ALPHANUMERIC) for _ in range(NONCE_LENGTH))

fresh_nonce = nonce

def now():
    return datetime.now(timezone.utc).strftime('%a, %d %b %Y %H:%M:%S GMT')

def split_url(url):
    path, _, query = url.partition('?')
    return path, query

def signature(method, path, query, nonce, date, content_type, secret):
    signed = '\n'.join([method, nonce, date, content_type, path, query, '']).lower()
    digest = hmac.new(secret.encode('utf-8'), signed.encode('utf-8'), hashlib.sha256).digest()
    return base64.b64encode(digest).decode('ascii')

def authorization(access, signature, nonce):
    return 'On ' + access + ':HmacSHA256:' + signature

def headers(method, url_path, query, access=None, secret=None, nonce=None, date=None,
            content_type='application/json'):
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
    try:
        return credentials()
    except NoCredentials:
        return None, None

class Client:

    def __init__(self, access=None, secret=None, base=BASE):
        self._access = access
        self._secret = secret
        self.base = base

    @classmethod
    def authenticated(cls, base=BASE):
        access, secret = credentials()
        return cls(access=access, secret=secret, base=base)

    @classmethod
    def configured(cls, base=BASE):
        access, secret = optional_credentials()
        return cls(access=access, secret=secret, base=base)

    @property
    def signed(self):
        return self._access is not None and self._secret is not None

    def __repr__(self):
        return '<Onshape ' + self.base + (' signed>' if self.signed else ' anonymous>')

    def get(self, url, accept='application/json'):
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
