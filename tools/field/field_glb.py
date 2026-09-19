#!/usr/bin/env python3
import argparse
import json
import math
import os
import re
import sys

import gltf
import onshape
from step_to_field import clean

INCH = 0.0254
GRID_IN = 0.05
AGREE_IN = 3.0
LEAST_COMPARED = 10
OUT = os.path.join(os.path.dirname(__file__), '..', '..', 'TeamCode', 'src', 'test', 'resources',
                   'org', 'firstinspires', 'ftc', 'teamcode', 'sim', 'field.glb')
COLLISION = os.path.join(os.path.dirname(OUT), 'field.json')

SKIP = re.compile(r'screw|fhts|nut\b|washer|rivet|rivnut|bolt|spacer|bearing|cable tie|plug|\bpin\b|'
                  r'damper|hinge|panel link|strap|clip|under tile|peanut|'
                  r'fastener|quick release|soft tiles|perimeter|rail|side glass', re.I)

KEEP = re.compile(r'field panel|ftc rail|side glass', re.I)

def to_field(p):
    return (p[1] / INCH, -p[0] / INCH, p[2] / INCH)

def snap(triangles, grid):
    out = []
    for a, b, c in triangles:
        at = tuple(tuple(round(v / grid) * grid for v in p) for p in (a, b, c))
        if at[0] == at[1] or at[1] == at[2] or at[0] == at[2]:
            continue
        out.append(at)
    return out

def visual_parts(parts, grid=GRID_IN):
    out = []
    for part in parts:
        if not KEEP.search(part.name) and any(
                SKIP.search(name) for name in list(part.path) + [part.name]):
            continue
        triangles = snap([tuple(to_field(p) for p in t) for t in part.triangles], grid)
        if triangles:
            out.append(gltf.Part(part.name, list(part.path), triangles, part.colour))
    return out

def _centre(part):
    points = [v for t in part.triangles for v in t]
    return tuple(sum(p[i] for p in points) / len(points) for i in range(3))

def disagreements(parts, collision, within=AGREE_IN, least=LEAST_COMPARED):
    blue = [_centre(p) for p in parts if re.search(r'blue', p.name, re.I)]
    red = [_centre(p) for p in parts if re.search(r'red', p.name, re.I)]
    if len(blue) < least or len(red) < least:
        return ['could not judge which way round the field is: %d blue parts and %d red, '
                'and at least %d of each are needed' % (len(blue), len(red), least)]

    off = []
    if not all(c[1] < 0 for c in blue) or not all(c[1] > 0 for c in red):
        off.append('the field is the wrong way round: blue belongs at negative y and red at '
                   'positive, and %d of %d blue parts and %d of %d red are on the wrong side'
                   % (sum(1 for c in blue if c[1] >= 0), len(blue),
                      sum(1 for c in red if c[1] <= 0), len(red)))

    half = collision['size'] / 2
    reach = max(abs(v[i]) for p in parts for t in p.triangles for v in t for i in (0, 1))
    if not half * 0.9 < reach < half * 3:
        off.append('the model reaches %.1f in from the middle; the collision model puts the walls '
                   'at %.1f in, so the frame or the units are wrong' % (reach, half))
    return off

def fetch():
    url = ('/api/v10/assemblies/d/' + onshape.FIELD_DOCUMENT + '/w/' + onshape.FIELD_WORKSPACE
           + '/e/' + onshape.FIELD_ASSEMBLY + '/gltf')
    return onshape.Client.configured().get(url, accept='model/gltf+json')

def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--export', help='a glTF export to build from, instead of fetching one')
    parser.add_argument('--grid', type=float, default=GRID_IN,
                        help='inches to snap points to (default: %(default)s)')
    parser.add_argument('--out', default=OUT)
    args = parser.parse_args(argv)

    if args.export:
        with open(args.export, 'rb') as f:
            export = f.read()
    else:
        try:
            export = fetch()
        except onshape.NoCredentials as refusal:
            print(refusal, file=sys.stderr)
            return 2
    try:
        read = gltf.read(export)
    except gltf.NotGltf as wrong:
        print('the export is not glTF this can read: %s' % wrong, file=sys.stderr)
        return 2

    parts = visual_parts(read, args.grid)
    if not parts:
        print('nothing was kept; the export named none of the parts we draw', file=sys.stderr)
        return 2

    with open(COLLISION) as f:
        collision = json.load(f)
    off = disagreements(parts, collision)
    if off:
        print('the visual model and the collision model disagree about where the field is:',
              file=sys.stderr)
        for line in off:
            print('  ' + line, file=sys.stderr)
        return 1

    body = gltf.write(parts)
    with open(args.out, 'wb') as f:
        f.write(body)
    before = sum(len(p.triangles) for p in read)
    after = sum(len(p.triangles) for p in parts)
    print('%s: %d parts, %d triangles of %d, %.1f MB'
          % (os.path.relpath(args.out), len(parts), after, before, len(body) / 1e6))
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
