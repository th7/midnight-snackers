#!/usr/bin/env python3
"""Fetch the field document's images: the panel artwork and the four goal April Tags.

These are blob elements in FIRST's Onshape document, which answers for them unsigned. They are
the part of the field that STEP cannot carry -- the geometry says where a panel is, and only
these say what is printed on it -- and the four tag images are what a simulated camera will
eventually have to show.

    python3 tools/field/assets.py --fetch

Nothing is written until every image has arrived and been checked. A run that wrote three
textures and stopped would leave a tree that looks fetched, and the missing one would turn up
later as a part rendered blank.

No dependencies beyond Python 3.
"""
import argparse
import json
import os
import sys

import onshape

# The images we take. Named exactly as the document names them, so that a rename upstream stops
# the run rather than quietly fetching less than it used to.
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
    """An image the pipeline expects is not in the document under that name any more."""


class NotAnImage(Exception):
    """What arrived is not a PNG. An error page saved under a .png would sit in the tree looking
    like a texture until something rendered it as noise."""


class BadName(Exception):
    """A name from the document that will not be used as a filename."""


def safe_name(name):
    """A document's element name as a filename, or a refusal. Names come from Onshape, and a name
    is about to become a path: one that climbs out of the destination must not be written."""
    if not name or name != name.strip():
        raise BadName('blank or padded element name: ' + repr(name))
    if os.path.isabs(name) or os.path.splitdrive(name)[0]:
        raise BadName('element name is an absolute path: ' + name)
    if name in ('.', '..') or '/' in name or '\\' in name or os.sep in name:
        raise BadName('element name is a path rather than a name: ' + name)
    return name


def blobs(client, document=DOCUMENT, workspace=WORKSPACE):
    """Every blob element in the document, as {name: element id}."""
    url = ('/api/v10/documents/d/' + document + '/w/' + workspace + '/elements')
    listed = json.loads(client.get(url).decode('utf-8'))
    return {element['name']: element['id']
            for element in listed if element.get('elementType') == 'BLOB'}


def download(client, element, document=DOCUMENT, workspace=WORKSPACE):
    """One blob, as bytes."""
    url = ('/api/v10/blobelements/d/' + document + '/w/' + workspace + '/e/' + element)
    return client.get(url, accept='application/octet-stream')


def fetch(client, document=DOCUMENT, workspace=WORKSPACE, into=OUT, names=TEXTURES):
    """Every image in `names`, into `into`. Returns the paths written.

    Everything is fetched and checked before anything is written, so a document that has lost one
    of them leaves the tree as it was rather than half updated."""
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

    # No credentials: the document answers for its blobs unsigned.
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
