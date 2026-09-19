"""Reading glTF: what a part is called, where its triangles are, and what colour it is.

Onshape exports the assembly as glTF, which is a published format, so these tests build glTF
byte for byte and read it back. Nothing here touches the network or Onshape.

The failure worth designing against is silent garbage: an accessor that runs off the end of its
buffer, or a component type nobody handled, must raise rather than return plausible-looking
triangles that are quietly wrong.
"""
import base64
import json
import math
import struct
import unittest

import gltf

FLOAT, UNSIGNED_BYTE, UNSIGNED_SHORT, UNSIGNED_INT = 5126, 5121, 5123, 5125
TRIANGLE = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def glb(document, buffer=b''):
    """A .glb container around a glTF document and its binary chunk, padded as the format says:
    the JSON chunk with spaces, the binary chunk with zeros, both to four bytes."""
    text = json.dumps(document).encode('utf-8')
    text += b' ' * (-len(text) % 4)
    buffer += b'\x00' * (-len(buffer) % 4)
    chunks = struct.pack('<II', len(text), 0x4E4F534A) + text
    if buffer:
        chunks += struct.pack('<II', len(buffer), 0x004E4942) + buffer
    return struct.pack('<4sII', b'glTF', 2, 12 + len(chunks)) + chunks


class Build:
    """A glTF document under construction: parts go in, bytes come out."""

    def __init__(self):
        self.doc = {'asset': {'version': '2.0'}, 'scenes': [{'nodes': []}], 'scene': 0,
                    'nodes': [], 'meshes': [], 'accessors': [], 'bufferViews': [], 'buffers': []}
        self.blob = b''

    def _view(self, data, stride=None):
        offset = len(self.blob)
        self.blob += data + b'\x00' * (-len(data) % 4)
        view = {'buffer': 0, 'byteOffset': offset, 'byteLength': len(data)}
        if stride:
            view['byteStride'] = stride
        self.doc['bufferViews'].append(view)
        return len(self.doc['bufferViews']) - 1

    def positions(self, points, stride=None, pad=0):
        """A VEC3 FLOAT accessor. `pad` bytes of filler after each point makes it interleaved."""
        data = b''.join(struct.pack('<fff', *p) + b'\x00' * pad for p in points)
        view = self._view(data, stride)
        self.doc['accessors'].append({'bufferView': view, 'componentType': FLOAT,
                                      'count': len(points), 'type': 'VEC3'})
        return len(self.doc['accessors']) - 1

    def indices(self, values, component=UNSIGNED_SHORT):
        fmt = {UNSIGNED_BYTE: '<B', UNSIGNED_SHORT: '<H', UNSIGNED_INT: '<I'}[component]
        data = b''.join(struct.pack(fmt, v) for v in values)
        view = self._view(data)
        self.doc['accessors'].append({'bufferView': view, 'componentType': component,
                                      'count': len(values), 'type': 'SCALAR'})
        return len(self.doc['accessors']) - 1

    def mesh(self, name, primitives):
        self.doc['meshes'].append({'name': name, 'primitives': primitives})
        return len(self.doc['meshes']) - 1

    def node(self, name=None, mesh=None, children=None, root=True, **placement):
        node = {}
        if name is not None:
            node['name'] = name
        if mesh is not None:
            node['mesh'] = mesh
        if children:
            node['children'] = children
        node.update(placement)
        self.doc['nodes'].append(node)
        index = len(self.doc['nodes']) - 1
        if root:
            self.doc['scenes'][0]['nodes'].append(index)
        return index

    def material(self, name, rgba):
        self.doc.setdefault('materials', []).append(
            {'name': name, 'pbrMetallicRoughness': {'baseColorFactor': list(rgba)}})
        return len(self.doc['materials']) - 1

    def bytes(self):
        self.doc['buffers'] = [{'byteLength': len(self.blob)}] if self.blob else []
        return glb(self.doc, self.blob)


def one_triangle(points=TRIANGLE, name='Part', node_name=None, **placement):
    """The simplest whole document: one indexed triangle in one node."""
    build = Build()
    position = build.positions(points)
    index = build.indices([0, 1, 2])
    mesh = build.mesh(name, [{'attributes': {'POSITION': position}, 'indices': index}])
    build.node(name=node_name if node_name is not None else name, mesh=mesh, **placement)
    return build


class Container(unittest.TestCase):
    def test_a_triangle_reads_back_as_itself(self):
        parts = gltf.read(one_triangle().bytes())
        self.assertEqual(1, len(parts))
        self.assertEqual('Part', parts[0].name)
        self.assertEqual([TRIANGLE], [tuple(t) for t in parts[0].triangles])

    def test_bytes_that_are_not_glb_are_refused(self):
        with self.assertRaises(gltf.NotGltf):
            gltf.read(b'<!doctype html><html>Onshape said no</html>')

    def test_a_version_we_do_not_read_is_refused(self):
        body = one_triangle().bytes()
        with self.assertRaises(gltf.NotGltf):
            gltf.read(body[:4] + struct.pack('<I', 1) + body[8:])

    def test_a_truncated_container_is_refused(self):
        with self.assertRaises(gltf.NotGltf):
            gltf.read(one_triangle().bytes()[:20])

    def test_plain_json_gltf_with_an_embedded_buffer_reads(self):
        """Onshape can answer with JSON rather than a .glb; a data: buffer is the same geometry."""
        build = one_triangle()
        body = build.bytes()
        del body
        build.doc['buffers'] = [{'byteLength': len(build.blob),
                                 'uri': 'data:application/octet-stream;base64,'
                                        + base64.b64encode(build.blob).decode('ascii')}]
        parts = gltf.read(json.dumps(build.doc).encode('utf-8'))
        self.assertEqual([TRIANGLE], [tuple(t) for t in parts[0].triangles])


class Placement(unittest.TestCase):
    def test_a_translation_moves_the_triangle(self):
        parts = gltf.read(one_triangle(translation=[10.0, 20.0, 30.0]).bytes())
        self.assertEqual([(10.0, 20.0, 30.0), (11.0, 20.0, 30.0), (10.0, 21.0, 30.0)],
                         [tuple(t) for t in parts[0].triangles[0]])

    def test_a_scale_and_a_rotation_are_applied_in_the_order_the_format_says(self):
        """glTF composes a node as translation * rotation * scale, so a scaled point is rotated
        and only then moved; any other order puts the part somewhere else entirely."""
        quarter_turn_about_z = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        parts = gltf.read(one_triangle(translation=[1.0, 0.0, 0.0], scale=[2.0, 2.0, 2.0],
                                       rotation=quarter_turn_about_z).bytes())
        got = [tuple(round(v, 6) for v in t) for t in parts[0].triangles[0]]
        self.assertEqual([(1.0, 0.0, 0.0), (1.0, 2.0, 0.0), (-1.0, 0.0, 0.0)], got)

    def test_a_matrix_is_read_column_major(self):
        """glTF writes a matrix down its columns. Read row-major, a translation lands in the wrong
        place and a shear appears out of nowhere."""
        matrix = [1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  5, 6, 7, 1]
        parts = gltf.read(one_triangle(matrix=[float(v) for v in matrix]).bytes())
        self.assertEqual((5.0, 6.0, 7.0), tuple(parts[0].triangles[0][0]))

    def test_transforms_compose_down_the_tree(self):
        """An assembly is nested, so a part's place is every transform above it, in order."""
        build = Build()
        position = build.positions(TRIANGLE)
        index = build.indices([0, 1, 2])
        mesh = build.mesh('Leaf', [{'attributes': {'POSITION': position}, 'indices': index}])
        leaf = build.node(name='Leaf', mesh=mesh, root=False, translation=[1.0, 0.0, 0.0])
        middle = build.node(name='Middle', children=[leaf], root=False, translation=[0.0, 2.0, 0.0])
        build.node(name='Root', children=[middle], translation=[0.0, 0.0, 3.0])
        parts = gltf.read(build.bytes())
        self.assertEqual((1.0, 2.0, 3.0), tuple(parts[0].triangles[0][0]))

    def test_a_node_without_a_mesh_still_places_its_children(self):
        build = Build()
        position = build.positions(TRIANGLE)
        index = build.indices([0, 1, 2])
        mesh = build.mesh('Leaf', [{'attributes': {'POSITION': position}, 'indices': index}])
        leaf = build.node(name='Leaf', mesh=mesh, root=False)
        build.node(name='Empty', children=[leaf], translation=[0.0, 0.0, 9.0])
        parts = gltf.read(build.bytes())
        self.assertEqual(1, len(parts))
        self.assertEqual((0.0, 0.0, 9.0), tuple(parts[0].triangles[0][0]))


class Accessors(unittest.TestCase):
    def test_indices_of_every_width_read_the_same(self):
        for component in (UNSIGNED_BYTE, UNSIGNED_SHORT, UNSIGNED_INT):
            with self.subTest(component):
                build = Build()
                position = build.positions(TRIANGLE)
                index = build.indices([0, 1, 2], component)
                mesh = build.mesh('Part', [{'attributes': {'POSITION': position}, 'indices': index}])
                build.node(name='Part', mesh=mesh)
                self.assertEqual([TRIANGLE], [tuple(t) for t in gltf.read(build.bytes())[0].triangles])

    def test_interleaved_positions_are_read_by_their_stride(self):
        """A byteStride wider than the element means other attributes sit between the points; a
        reader that ignores it walks into them and reports nonsense coordinates."""
        build = Build()
        position = build.positions(TRIANGLE, stride=16, pad=4)
        index = build.indices([0, 1, 2])
        mesh = build.mesh('Part', [{'attributes': {'POSITION': position}, 'indices': index}])
        build.node(name='Part', mesh=mesh)
        self.assertEqual([TRIANGLE], [tuple(t) for t in gltf.read(build.bytes())[0].triangles])

    def test_a_primitive_without_indices_takes_its_points_in_order(self):
        build = Build()
        position = build.positions(TRIANGLE)
        mesh = build.mesh('Part', [{'attributes': {'POSITION': position}}])
        build.node(name='Part', mesh=mesh)
        self.assertEqual([TRIANGLE], [tuple(t) for t in gltf.read(build.bytes())[0].triangles])

    def test_an_accessor_that_runs_past_its_buffer_is_refused(self):
        """Silent garbage is the failure to design out: a short buffer must stop the read, not
        yield triangles that look plausible and are wrong."""
        build = one_triangle()
        build.doc['accessors'][0]['count'] = 400
        with self.assertRaises(gltf.NotGltf):
            gltf.read(build.bytes())

    def test_a_component_type_we_do_not_know_is_refused(self):
        build = one_triangle()
        build.doc['accessors'][1]['componentType'] = 5130
        with self.assertRaises(gltf.NotGltf):
            gltf.read(build.bytes())

    def test_an_index_past_the_end_of_the_points_is_refused(self):
        build = Build()
        position = build.positions(TRIANGLE)
        index = build.indices([0, 1, 7])
        mesh = build.mesh('Part', [{'attributes': {'POSITION': position}, 'indices': index}])
        build.node(name='Part', mesh=mesh)
        with self.assertRaises(gltf.NotGltf):
            gltf.read(build.bytes())


class Naming(unittest.TestCase):
    def test_a_part_is_named_by_its_node_rather_than_its_mesh(self):
        """Onshape names the node for the instance and the mesh for the shape, and the pipeline
        filters and groups by instance name -- 'am-5855 Flower Assembly <2>', not 'HIPS Pipe'."""
        parts = gltf.read(one_triangle(name='Shape', node_name='Instance <2>').bytes())
        self.assertEqual('Instance <2>', parts[0].name)

    def test_a_part_with_no_name_anywhere_still_reads(self):
        build = Build()
        position = build.positions(TRIANGLE)
        index = build.indices([0, 1, 2])
        mesh = build.mesh(None, [{'attributes': {'POSITION': position}, 'indices': index}])
        del build.doc['meshes'][0]['name']
        build.node(mesh=mesh)
        parts = gltf.read(build.bytes())
        self.assertEqual(1, len(parts))
        self.assertEqual('', parts[0].name)

    def test_a_part_carries_the_names_of_the_nodes_above_it(self):
        """The hive's parts are found by the assembly they sit in, which is a node further up."""
        build = Build()
        position = build.positions(TRIANGLE)
        index = build.indices([0, 1, 2])
        mesh = build.mesh('Pipe', [{'attributes': {'POSITION': position}, 'indices': index}])
        leaf = build.node(name='Flower HIPS Pipe', mesh=mesh, root=False)
        build.node(name='am-5855 Flower Assembly <1>', children=[leaf])
        parts = gltf.read(build.bytes())
        self.assertEqual(['am-5855 Flower Assembly <1>', 'Flower HIPS Pipe'], parts[0].path)


class Colour(unittest.TestCase):
    def test_a_base_colour_becomes_a_hex_string(self):
        build = one_triangle()
        material = build.material('Green', [0.372549, 0.654902, 0.239216, 1.0])
        build.doc['meshes'][0]['primitives'][0]['material'] = material
        self.assertEqual('#5fa73d', gltf.read(build.bytes())[0].colour)

    def test_a_part_with_no_material_has_no_colour(self):
        self.assertIsNone(gltf.read(one_triangle().bytes())[0].colour)

    def test_each_primitive_of_a_mesh_keeps_its_own_colour(self):
        """One Onshape part can carry two appearances; merging them would lose one."""
        build = Build()
        position = build.positions(TRIANGLE)
        index = build.indices([0, 1, 2])
        red = build.material('Red', [1.0, 0.0, 0.0, 1.0])
        blue = build.material('Blue', [0.0, 0.0, 1.0, 1.0])
        mesh = build.mesh('Part', [
            {'attributes': {'POSITION': position}, 'indices': index, 'material': red},
            {'attributes': {'POSITION': position}, 'indices': index, 'material': blue}])
        build.node(name='Part', mesh=mesh)
        self.assertEqual(['#ff0000', '#0000ff'], [p.colour for p in gltf.read(build.bytes())])


if __name__ == '__main__':
    unittest.main()


class Writing(unittest.TestCase):
    """Writing glTF, checked by reading it back: the reader is tested against bytes built by hand
    above, so a round trip through it says the writer agrees with the format and not merely with
    itself."""

    def roundtrip(self, parts):
        return gltf.read(gltf.write(parts))

    def test_a_triangle_survives_a_round_trip(self):
        part = gltf.Part('Part', ['Root', 'Part'], [TRIANGLE], '#5fa73d')
        back = self.roundtrip([part])
        self.assertEqual(1, len(back))
        self.assertEqual('Part', back[0].name)
        self.assertEqual(['Root', 'Part'], back[0].path)
        self.assertEqual([TRIANGLE], [tuple(t) for t in back[0].triangles])
        self.assertEqual('#5fa73d', back[0].colour)

    def test_what_is_written_is_a_glb(self):
        body = gltf.write([gltf.Part('Part', ['Part'], [TRIANGLE], None)])
        self.assertEqual(b'glTF', body[:4])
        self.assertEqual(len(body), struct.unpack_from('<I', body, 8)[0])
        self.assertEqual(0, len(body) % 4, 'the chunks are padded to four bytes')

    def test_parts_keep_the_tree_their_paths_describe(self):
        """A renderer groups by the assembly a part sits in, so the nesting has to survive."""
        parts = [
            gltf.Part('Pipe', ['Field', 'Flower <1>', 'Pipe'], [TRIANGLE], '#5fa73d'),
            gltf.Part('Backstop', ['Field', 'Flower <1>', 'Backstop'], [TRIANGLE], '#641c65'),
            gltf.Part('Tray', ['Field', 'Tray'], [TRIANGLE], '#666666'),
        ]
        back = self.roundtrip(parts)
        self.assertEqual(sorted(p.path for p in parts), sorted(p.path for p in back))

    def test_two_parts_of_the_same_colour_share_one_material(self):
        body = gltf.write([gltf.Part('A', ['A'], [TRIANGLE], '#5fa73d'),
                           gltf.Part('B', ['B'], [TRIANGLE], '#5fa73d'),
                           gltf.Part('C', ['C'], [TRIANGLE], '#641c65')])
        document, _ = gltf._unpack_glb(body)
        self.assertEqual(2, len(document['materials']))

    def test_a_part_with_no_colour_round_trips_without_one(self):
        self.assertIsNone(self.roundtrip([gltf.Part('P', ['P'], [TRIANGLE], None)])[0].colour)

    def test_positions_carry_the_bounds_the_format_requires(self):
        """glTF requires min and max on a POSITION accessor; a viewer uses them to frame the scene
        and some refuse the file without them."""
        body = gltf.write([gltf.Part('P', ['P'], [TRIANGLE], None)])
        document, _ = gltf._unpack_glb(body)
        position = next(a for a in document['accessors'] if a['type'] == 'VEC3')
        self.assertEqual([0.0, 0.0, 0.0], position['min'])
        self.assertEqual([1.0, 1.0, 0.0], position['max'])

    def test_vertices_shared_between_triangles_are_written_once(self):
        """Two triangles sharing an edge are four points, not six; at the size of a field that
        difference is megabytes."""
        second = (TRIANGLE[1], (1.0, 1.0, 0.0), TRIANGLE[2])
        body = gltf.write([gltf.Part('P', ['P'], [TRIANGLE, second], None)])
        document, _ = gltf._unpack_glb(body)
        position = next(a for a in document['accessors'] if a['type'] == 'VEC3')
        self.assertEqual(4, position['count'])

    def test_nothing_written_is_empty(self):
        """A part whose triangles all fell away is not written at all, rather than written as a
        mesh with no primitive, which is a file some readers reject."""
        back = self.roundtrip([gltf.Part('Gone', ['Gone'], [], None),
                               gltf.Part('Here', ['Here'], [TRIANGLE], None)])
        self.assertEqual(['Here'], [p.name for p in back])
