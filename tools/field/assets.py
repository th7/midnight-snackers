#!/usr/bin/env python3
import argparse
import json
import os
import sys

import onshape

TEXTURES = (
    'BIOBUZZ_Panel_Resized.png',
    'GoalAprilTag_blueaudience.png',
    'GoalAprilTag_bluescoring.png',
    'GoalAprilTag_redaudience.png',
    'GoalAprilTag_redscoring.png',
)
PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'
DOCUMENT = onshape.FIELD_DOCUMENT
WORKSPACE = onshape.FIELD_WORKSPACE
OUT = os.path.join(os.path.dirname(__file__), '..', '..', 'TeamCode', 'src', 'test', 'resources',
                   'org', 'firstinspires', 'ftc', 'teamcode', 'sim', 'textures')

class Missing(Exception):
    pass

class NotAnImage(Exception):
    pass

class BadName(Exception):
    pass

def safe_name(name):
    if not name or name != name.strip():
        raise BadName('blank or padded element name: ' + repr(name))
    if os.path.isabs(name) or os.path.splitdrive(name)[0]:
        raise BadName('element name is an absolute path: ' + name)
    if name in ('.', '..') or '/' in name or '\\' in name or os.sep in name:
        raise BadName('element name is a path rather than a name: ' + name)
    return name

def blobs(client, document=DOCUMENT, workspace=WORKSPACE):
    url = ('/api/v10/documents/d/' + document + '/w/' + workspace + '/elements')
    listed = json.loads(client.get(url).decode('utf-8'))
    return {element['name']: element['id']
            for element in listed if element.get('elementType') == 'BLOB'}

def download(client, element, document=DOCUMENT, workspace=WORKSPACE):
    url = ('/api/v10/blobelements/d/' + document + '/w/' + workspace + '/e/' + element)
    return client.get(url, accept='application/octet-stream')

def fetch(client, document=DOCUMENT, workspace=WORKSPACE, into=OUT, names=TEXTURES):
    available = blobs(client, document, workspace)
    absent = [name for name in names if name not in available]
    if absent:
        raise Missing('the field document has no blob named ' + ', '.join(absent)
                      + '. It holds: ' + ', '.join(sorted(available)) or '(nothing)')

    fetched = {}
    for name in names:
        body = download(client, available[name], document, workspace)
        if not body:
            raise NotAnImage(name + ' came back empty')
        if not body.startswith(PNG_SIGNATURE):
            raise NotAnImage(name + ' is not a PNG; it begins ' + repr(body[:16]))
        fetched[safe_name(name)] = body

    os.makedirs(into, exist_ok=True)
    written = []
    for name, body in fetched.items():
        path = os.path.join(into, name)
        with open(path, 'wb') as out:
            out.write(body)
        written.append(path)
    return written

def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--fetch', action='store_true', help='fetch the images')
    parser.add_argument('--into', default=OUT, help='where to write them (default: %(default)s)')
    parser.add_argument('--document', default=DOCUMENT)
    parser.add_argument('--workspace', default=WORKSPACE)
    args = parser.parse_args(argv)
    if not args.fetch:
        parser.error('nothing to do; --fetch fetches the images')

    client = onshape.Client()
    try:
        written = fetch(client, args.document, args.workspace, args.into)
    except (Missing, NotAnImage, BadName) as refusal:
        print(refusal, file=sys.stderr)
        return 2
    for path in written:
        print(os.path.relpath(path), os.path.getsize(path), 'bytes')
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
