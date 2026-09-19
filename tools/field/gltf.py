#!/usr/bin/env python3
"""A reader for the glTF 2.0 that Onshape exports an assembly as.

Onshape answers with either a .glb -- a small binary container around a JSON document and one
blob -- or the JSON on its own with its buffer inline as a data: URI. Either way what comes out
here is a flat list of parts: each one its node's name, the names of the nodes above it, its
triangles already placed in the assembly's frame, and its colour.

Only what the field pipeline needs is read. Everything the format allows and this does not
handle raises rather than being skipped: a part silently dropped, or one whose triangles are
read at the wrong stride, would reach the simulator as geometry that looks plausible and is
wrong, which is the one outcome worth spending code to prevent.

No dependencies beyond Python 3.
"""
import base64
import json
import struct

GLB_MAGIC = b'glTF'
GLB_VERSION = 2
CHUNK_JSON, CHUNK_BIN = 0x4E4F534A, 0x004E4942

# componentType -> (struct format, bytes). glTF allows signed widths for attributes; indices are
# always unsigned, and positions are always float, so these are what a mesh can hold.
COMPONENTS = {
    5120: ('<b', 1), 5121: ('<B', 1), 5122: ('<h', 2),
    5123: ('<H', 2), 5125: ('<I', 4), 5126: ('<f', 4),
}
COUNTS = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
TRIANGLES = 4  # primitive mode; the only one a solid export uses


class NotGltf(Exception):
    """The bytes are not glTF we can read, or they are glTF that says something we will not guess
    at. Raised in place of returning geometry that cannot be trusted."""


class Part:
    """One primitive of one mesh, where the assembly puts it.

    `name` is the node's, which for an Onshape export is the instance ('Flower HIPS Pipe <2>');
    `path` is every named node from the scene root down to it, which is how a part is found by
    the assembly it belongs to. `triangles` are triples of (x, y, z) in the assembly's frame and
    the file's units, which for glTF are metres.
    """

    __slots__ = ('name', 'path', 'triangles', 'colour')

    def __init__(self, name, path, triangles, colour):
        self.name = name
        self.path = path
        self.triangles = triangles
        self.colour = colour

    def __repr__(self):
        return '<Part ' + (self.name or '(unnamed)') + ' ' + str(len(self.triangles)) + ' triangles>'


# --- the container ---------------------------------------------------------------------------

def _unpack_glb(data):
    """A .glb into its document and its binary chunk."""
    if len(data) < 12:
        raise NotGltf('too short to be a glb: ' + str(len(data)) + ' bytes')
    magic, version, length = struct.unpack_from('<4sII', data, 0)
    if magic != GLB_MAGIC:
        raise NotGltf('not a glb: it begins ' + repr(data[:4]))
    if version != GLB_VERSION:
        raise NotGltf('glb version ' + str(version) + '; this reads version ' + str(GLB_VERSION))
    if length > len(data):
        raise NotGltf('glb says it is ' + str(length) + ' bytes and ' + str(len(data)) + ' arrived')
    document, blob, at = None, b'', 12
    while at + 8 <= length:
        size, kind = struct.unpack_from('<II', data, at)
        at += 8
        if at + size > length:
            raise NotGltf('a glb chunk runs past the end of the file')
        chunk = data[at:at + size]
        if kind == CHUNK_JSON and document is None:
            document = json.loads(chunk.decode('utf-8'))
        elif kind == CHUNK_BIN and not blob:
            blob = chunk
        at += size + (-size % 4)
    if document is None:
        raise NotGltf('the glb has no JSON chunk')
    return document, blob


def _buffers(document, blob):
    """Every buffer the document declares, as bytes. A buffer with no uri is the glb's own blob;
    a data: uri carries its own. A uri pointing at a file beside the document is refused: we
    fetch one response, and a reader that quietly returned short buffers would be worse."""
    out = []
    for index, buffer in enumerate(document.get('buffers', [])):
        uri = buffer.get('uri')
        if uri is None:
            out.append(blob)
        elif uri.startswith('data:'):
            _, _, encoded = uri.partition(',')
            try:
                out.append(base64.b64decode(encoded))
            except Exception:
                raise NotGltf('buffer ' + str(index) + ' has a data uri that is not base64') from None
        else:
            raise NotGltf('buffer ' + str(index) + ' is a separate file (' + uri[:60]
                          + '); ask Onshape for a glb or for buffers inline')
    return out


# --- accessors -------------------------------------------------------------------------------

def _read_accessor(document, buffers, index):
    """One accessor as a list of tuples, honouring its buffer view's stride."""
    accessors = document.get('accessors', [])
    if not 0 <= index < len(accessors):
        raise NotGltf('accessor ' + str(index) + ' does not exist')
    accessor = accessors[index]
    if 'sparse' in accessor:
        raise NotGltf('accessor ' + str(index) + ' is sparse, which this does not read')
    if 'bufferView' not in accessor:
        raise NotGltf('accessor ' + str(index) + ' has no buffer view')
    if accessor['componentType'] not in COMPONENTS:
        raise NotGltf('accessor ' + str(index) + ' has component type '
                      + str(accessor['componentType']) + ', which this does not read')
    if accessor.get('type') not in COUNTS:
        raise NotGltf('accessor ' + str(index) + ' has type ' + str(accessor.get('type')))

    fmt, width = COMPONENTS[accessor['componentType']]
    per = COUNTS[accessor['type']]
    element = width * per

    views = document.get('bufferViews', [])
    if not 0 <= accessor['bufferView'] < len(views):
        raise NotGltf('accessor ' + str(index) + ' names a buffer view that does not exist')
    view = views[accessor['bufferView']]
    if not 0 <= view.get('buffer', 0) < len(buffers):
        raise NotGltf('a buffer view names a buffer that does not exist')
    data = buffers[view['buffer']]
    stride = view.get('byteStride') or element
    start = view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
    count = accessor['count']

    last = start + stride * (count - 1) + element if count else start
    if count and (start < 0 or last > view.get('byteOffset', 0) + view['byteLength']
                  or last > len(data)):
        raise NotGltf('accessor ' + str(index) + ' reads ' + str(count) + ' elements past the end '
                      'of its buffer; the export is truncated or the accessor is wrong')

    out = []
    for i in range(count):
        at = start + i * stride
        out.append(tuple(struct.unpack_from(fmt, data, at + c * width)[0] for c in range(per)))
    return out


# --- placement -------------------------------------------------------------------------------

IDENTITY = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def multiply(a, b):
    """a * b, both column-major as glTF writes them: element (row, col) is m[col * 4 + row]."""
    out = [0.0] * 16
    for col in range(4):
        for row in range(4):
            out[col * 4 + row] = sum(a[k * 4 + row] * b[col * 4 + k] for k in range(4))
    return tuple(out)


def from_trs(translation, rotation, scale):
    """glTF composes a node as translation * rotation * scale, in that order. Any other order
    puts a part that is both turned and scaled somewhere else entirely."""
    x, y, z, w = rotation
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
         (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
         (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))
    sx, sy, sz = scale
    return (r[0][0] * sx, r[0][1] * sx, r[0][2] * sx, 0.0,
            r[1][0] * sy, r[1][1] * sy, r[1][2] * sy, 0.0,
            r[2][0] * sz, r[2][1] * sz, r[2][2] * sz, 0.0,
            translation[0], translation[1], translation[2], 1.0)


def placement(node):
    """Where a node puts what hangs below it: its matrix, or its translation, rotation and scale."""
    if 'matrix' in node:
        matrix = node['matrix']
        if len(matrix) != 16:
            raise NotGltf('a node matrix has ' + str(len(matrix)) + ' numbers rather than 16')
        return tuple(float(v) for v in matrix)
    if not any(k in node for k in ('translation', 'rotation', 'scale')):
        return IDENTITY
    return from_trs(node.get('translation', (0.0, 0.0, 0.0)),
                    node.get('rotation', (0.0, 0.0, 0.0, 1.0)),
                    node.get('scale', (1.0, 1.0, 1.0)))


def apply(matrix, point):
    """A point through a column-major matrix."""
    x, y, z = point[0], point[1], point[2]
    return (matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12],
            matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13],
            matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14])


# --- colour ----------------------------------------------------------------------------------

def hex_colour(factor):
    """A material's base colour as '#rrggbb'.

    glTF defines baseColorFactor in linear space and the hex the rest of the pipeline uses is
    sRGB, so whether a transfer function belongs here was an open question. It does not: Onshape
    writes the appearance's sRGB value straight into the factor.

    Settled by exporting the field assembly and comparing every part the STEP pipeline had
    already coloured. Of 71 parts named by both, 67 agree exactly, and the four that differ are
    the April Tag plates, which `step_to_field.colour_for` deliberately overrides to #4a4a4a
    (the CAD has them white). A transfer function here would have moved all 67.
    """
    return '#' + ''.join('%02x' % max(0, min(255, round(c * 255))) for c in factor[:3])


def _colour_of(document, primitive):
    index = primitive.get('material')
    if index is None:
        return None
    materials = document.get('materials', [])
    if not 0 <= index < len(materials):
        raise NotGltf('a primitive names material ' + str(index) + ', which does not exist')
    factor = materials[index].get('pbrMetallicRoughness', {}).get('baseColorFactor')
    return hex_colour(factor) if factor else None


# --- the walk --------------------------------------------------------------------------------

def _primitive_parts(document, buffers, primitive, name, path, matrix):
    if primitive.get('mode', TRIANGLES) != TRIANGLES:
        raise NotGltf('primitive of ' + (name or '(unnamed)') + ' has mode '
                      + str(primitive.get('mode')) + '; a solid export is triangles')
    position = primitive.get('attributes', {}).get('POSITION')
    if position is None:
        raise NotGltf('primitive of ' + (name or '(unnamed)') + ' has no POSITION')
    points = [apply(matrix, p) for p in _read_accessor(document, buffers, position)]

    if 'indices' in primitive:
        order = [i[0] for i in _read_accessor(document, buffers, primitive['indices'])]
    else:
        order = list(range(len(points)))
    if len(order) % 3:
        raise NotGltf('primitive of ' + (name or '(unnamed)') + ' has ' + str(len(order))
                      + ' vertices, which is not whole triangles')
    for i in order:
        if not 0 <= i < len(points):
            raise NotGltf('primitive of ' + (name or '(unnamed)') + ' indexes point ' + str(i)
                          + ' of ' + str(len(points)))
    triangles = [(points[order[i]], points[order[i + 1]], points[order[i + 2]])
                 for i in range(0, len(order), 3)]
    return Part(name, path, triangles, _colour_of(document, primitive))


def read(data):
    """Every part of the glTF in `data`, placed in the assembly's frame."""
    if data[:4] == GLB_MAGIC:
        document, blob = _unpack_glb(data)
    else:
        try:
            document, blob = json.loads(data.decode('utf-8')), b''
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise NotGltf('not a glb and not JSON; it begins ' + repr(data[:24])) from None
    if not isinstance(document, dict) or 'asset' not in document:
        raise NotGltf('JSON that is not a glTF document: no asset')

    buffers = _buffers(document, blob)
    nodes = document.get('nodes', [])
    meshes = document.get('meshes', [])
    scenes = document.get('scenes', [])
    scene = document.get('scene', 0)
    if not scenes:
        raise NotGltf('the glTF has no scene')
    if not 0 <= scene < len(scenes):
        raise NotGltf('the glTF names scene ' + str(scene) + ', which does not exist')

    parts = []
    seen = set()

    def walk(index, parent, path):
        if not 0 <= index < len(nodes):
            raise NotGltf('a node names child ' + str(index) + ', which does not exist')
        if index in seen:
            raise NotGltf('node ' + str(index) + ' is reached twice; the scene is not a tree')
        seen.add(index)
        node = nodes[index]
        matrix = multiply(parent, placement(node))
        name = node.get('name', '')
        below = path + [name] if name else path
        if 'mesh' in node:
            if not 0 <= node['mesh'] < len(meshes):
                raise NotGltf('node ' + (name or str(index)) + ' names a mesh that does not exist')
            for primitive in meshes[node['mesh']].get('primitives', []):
                parts.append(_primitive_parts(document, buffers, primitive, name, below, matrix))
        for child in node.get('children', []):
            walk(child, matrix, below)

    for root in scenes[scene].get('nodes', []):
        walk(root, IDENTITY, [])
    return parts


# --- writing ---------------------------------------------------------------------------------

def _colour_factor(colour):
    """'#rrggbb' -> the linear-looking factor Onshape puts there, which is what `hex_colour`
    reads back: the two are inverses, and neither applies a transfer function."""
    return [int(colour[i:i + 2], 16) / 255 for i in (1, 3, 5)] + [1.0]


def write(parts, generator='midnight-snackers tools/field'):
    """`parts` as a .glb.

    Each part becomes one mesh of one primitive under a node, and the nodes are nested to match
    the paths, so that what `read` gives back is what went in -- names, tree, triangles and
    colours alike. Points shared between a part's triangles are written once and indexed.
    """
    document = {
        'asset': {'version': '2.0', 'generator': generator},
        'scene': 0, 'scenes': [{'nodes': []}],
        'nodes': [], 'meshes': [], 'materials': [],
        'accessors': [], 'bufferViews': [], 'buffers': [],
    }
    blob = bytearray()
    materials = {}

    def view(data, target):
        blob.extend(b'\x00' * (-len(blob) % 4))
        offset = len(blob)
        blob.extend(data)
        document['bufferViews'].append(
            {'buffer': 0, 'byteOffset': offset, 'byteLength': len(data), 'target': target})
        return len(document['bufferViews']) - 1

    def material_for(colour):
        if colour not in materials:
            document['materials'].append(
                {'name': colour,
                 'pbrMetallicRoughness': {'baseColorFactor': _colour_factor(colour),
                                          'metallicFactor': 0.0, 'roughnessFactor': 0.7}})
            materials[colour] = len(document['materials']) - 1
        return materials[colour]

    # The nodes named by a path, so that parts of one assembly hang under one node. Only the
    # branches are shared: a leaf is always its own node, or two parts alike would collide.
    branches = {}

    def branch(path):
        parent = None
        for depth in range(len(path)):
            key = tuple(path[:depth + 1])
            if key not in branches:
                document['nodes'].append({'name': path[depth]})
                branches[key] = len(document['nodes']) - 1
                if parent is None:
                    document['scenes'][0]['nodes'].append(branches[key])
                else:
                    document['nodes'][parent].setdefault('children', []).append(branches[key])
            parent = branches[key]
        return parent

    for part in parts:
        if not part.triangles:
            continue  # a mesh with no primitive is a file some readers refuse
        order, points = [], {}
        for triangle in part.triangles:
            for point in triangle:
                key = tuple(point)
                if key not in points:
                    points[key] = len(points)
                order.append(points[key])

        data = b''.join(struct.pack('<fff', *p) for p in points)
        lows = [min(p[i] for p in points) for i in range(3)]
        highs = [max(p[i] for p in points) for i in range(3)]
        document['accessors'].append(
            {'bufferView': view(data, 34962), 'componentType': 5126, 'count': len(points),
             'type': 'VEC3', 'min': lows, 'max': highs})
        position = len(document['accessors']) - 1

        narrow = len(points) <= 0xFFFF
        fmt, component = ('<H', 5123) if narrow else ('<I', 5125)
        data = b''.join(struct.pack(fmt, i) for i in order)
        document['accessors'].append(
            {'bufferView': view(data, 34963), 'componentType': component, 'count': len(order),
             'type': 'SCALAR'})

        primitive = {'attributes': {'POSITION': position},
                     'indices': len(document['accessors']) - 1, 'mode': TRIANGLES}
        if part.colour:
            primitive['material'] = material_for(part.colour)
        document['meshes'].append({'name': part.name, 'primitives': [primitive]})

        path = list(part.path) if part.path else [part.name]
        node = {'name': path[-1], 'mesh': len(document['meshes']) - 1}
        document['nodes'].append(node)
        index = len(document['nodes']) - 1
        parent = branch(path[:-1]) if len(path) > 1 else None
        if parent is None:
            document['scenes'][0]['nodes'].append(index)
        else:
            document['nodes'][parent].setdefault('children', []).append(index)

    if not document['materials']:
        del document['materials']
    document['buffers'] = [{'byteLength': len(blob)}] if blob else []

    text = json.dumps(document, separators=(',', ':')).encode('utf-8')
    text += b' ' * (-len(text) % 4)
    blob.extend(b'\x00' * (-len(blob) % 4))
    chunks = struct.pack('<II', len(text), CHUNK_JSON) + text
    if blob:
        chunks += struct.pack('<II', len(blob), CHUNK_BIN) + bytes(blob)
    return struct.pack('<4sII', GLB_MAGIC, GLB_VERSION, 12 + len(chunks)) + chunks
