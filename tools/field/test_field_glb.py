import math
import unittest

import field_glb
import gltf

IN = 0.0254

def box(centre, size=1.0):
    x, y, z = centre
    h = size / 2
    corners = [(x + a * h, y + b * h, z + c * h)
               for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    out = []
    for a, b, c, d in quads:
        out.append((corners[a], corners[b], corners[c]))
        out.append((corners[a], corners[c], corners[d]))
    return out

class Frame(unittest.TestCase):

    def test_a_point_turns_a_quarter_and_becomes_inches(self):
        self.assertEqual((2.0, -1.0, 3.0),
                         tuple(round(c, 6) for c in field_glb.to_field((1 * IN, 2 * IN, 3 * IN))))

    def test_the_turn_is_the_one_that_puts_blue_at_negative_y(self):
        self.assertLess(field_glb.to_field((1.0, 0.0, 0.0))[1], 0)

    def test_the_floor_stays_the_floor(self):
        self.assertEqual(0.0, field_glb.to_field((1.0, 2.0, 0.0))[2])

class Keeping(unittest.TestCase):
    def test_hardware_is_dropped_by_name(self):
        parts = [gltf.Part('am-1042 10-32 Nylock Nut', ['Field', 'Nut <1>', 'am-1042 10-32 Nylock Nut'],
                           box((0, 0, 0)), '#888888'),
                 gltf.Part('Flower HIPS Pipe', ['Field', 'Flower <1>', 'Flower HIPS Pipe'],
                           box((0, 0, 0)), '#5fa73d')]
        kept = field_glb.visual_parts(parts)
        self.assertEqual(['Flower HIPS Pipe'], [p.name for p in kept])

    def test_a_part_inside_an_assembly_we_drop_goes_with_it(self):
        parts = [gltf.Part('Tile Corner', ['Field', 'am-2499: Soft Tiles <1>', 'Tile Corner'],
                           box((0, 0, 0)), '#333333')]
        self.assertEqual([], field_glb.visual_parts(parts))

    def test_the_wall_is_kept_though_its_name_says_rivet(self):
        parts = [gltf.Part('am-2556a: FTC Rail with Rivet Holes',
                           ['Field', 'am-0481b FTC Perimeter RevB <1>', 'am-2556a: FTC Rail with Rivet Holes'],
                           box((0, 0, 0)), '#cccccc'),
                 gltf.Part('FTC Field Side Glass 11in',
                           ['Field', 'am-0481b FTC Perimeter RevB <1>', 'FTC Field Side Glass 11in'],
                           box((0, 0, 0)), None),
                 gltf.Part('187 pop rivet',
                           ['Field', 'am-0481b FTC Perimeter RevB <1>', '187 pop rivet'],
                           box((0, 0, 0)), '#888888')]
        kept = [p.name for p in field_glb.visual_parts(parts)]

        self.assertIn('am-2556a: FTC Rail with Rivet Holes', kept)
        self.assertIn('FTC Field Side Glass 11in', kept)
        self.assertNotIn('187 pop rivet', kept, 'the rivets themselves are still hardware')

    def test_what_is_kept_is_put_in_the_field_frame(self):
        parts = [gltf.Part('Pipe', ['Field', 'Pipe'], box((1.0, 0.0, 0.5)), '#5fa73d')]
        kept = field_glb.visual_parts(parts)
        centre = [sum(v[i] for t in kept[0].triangles for v in t) / (3 * len(kept[0].triangles))
                  for i in range(3)]
        self.assertAlmostEqual(0.0, centre[0], places=1)
        self.assertAlmostEqual(-1.0 / IN, centre[1], places=1)
        self.assertAlmostEqual(0.5 / IN, centre[2], places=1)

class Snapping(unittest.TestCase):

    def test_no_point_moves_further_than_the_grid_can_move_it(self):
        triangles = box((0.3, 0.2, 0.1), 0.4)
        grid = 0.05
        snapped = field_glb.snap([tuple(tuple(c / IN for c in v) for v in t) for t in triangles], grid)
        before = {v for t in triangles for v in t}
        for triangle in snapped:
            for point in triangle:
                nearest = min(math.dist(point, tuple(c / IN for c in p)) for p in before)
                self.assertLessEqual(nearest, grid * math.sqrt(3) / 2 + 1e-9)

    def test_a_shape_coarser_than_the_grid_keeps_every_triangle(self):
        triangles = [tuple(tuple(c / IN for c in v) for v in t) for t in box((0, 0, 0), 1.0)]
        self.assertEqual(len(triangles), len(field_glb.snap(triangles, 0.05)))

    def test_triangles_that_collapse_onto_themselves_are_dropped(self):
        flat = [((0.0, 0.0, 0.0), (0.001, 0.0, 0.0), (0.0, 0.001, 0.0))]
        self.assertEqual([], field_glb.snap(flat, 0.05))

    def test_a_dense_shape_loses_triangles_without_losing_its_size(self):
        fine = []
        steps = 60
        for i in range(steps):
            a, b = 2 * math.pi * i / steps, 2 * math.pi * (i + 1) / steps
            for z in (0.0, 10.0):
                fine.append(((10 * math.cos(a), 10 * math.sin(a), z),
                             (10 * math.cos(b), 10 * math.sin(b), z), (0.0, 0.0, z)))
        snapped = field_glb.snap(fine, 1.0)
        self.assertLess(len(snapped), len(fine))
        reach = max(math.hypot(v[0], v[1]) for t in snapped for v in t)
        self.assertAlmostEqual(10.0, reach, delta=1.0)

class Agreeing(unittest.TestCase):

    COLLISION = {'size': 141.17, 'elements': []}

    def field(self, blue_at, red_at, reach=70.0):
        def export(x, y, z):
            return (-y * IN, x * IN, z * IN)
        parts = []
        for i in range(12):
            parts.append(gltf.Part('Blue Cell %d' % i, ['Field', 'Blue Cell %d' % i],
                                   box(export(0.0, blue_at, 40.0), 0.05), '#1651b0'))
            parts.append(gltf.Part('Red Cell %d' % i, ['Field', 'Red Cell %d' % i],
                                   box(export(0.0, red_at, 40.0), 0.05), '#c62828'))
        parts.append(gltf.Part('Wall', ['Field', 'Wall'], box(export(reach, 0.0, 6.0), 0.05), None))
        return field_glb.visual_parts(parts)

    def test_a_field_the_right_way_round_and_the_right_size_agrees(self):
        self.assertEqual([], field_glb.disagreements(self.field(-12.76, 12.74), self.COLLISION))

    def test_a_field_turned_the_wrong_way_is_caught(self):
        said = field_glb.disagreements(self.field(12.76, -12.74), self.COLLISION)
        self.assertEqual(1, len(said))
        self.assertIn('wrong way round', said[0])

    def test_a_model_left_in_metres_is_caught(self):
        tiny = [gltf.Part(p.name, p.path, [tuple(tuple(c * IN for c in v) for v in t)
                                           for t in p.triangles], p.colour)
                for p in self.field(-12.76, 12.74)]
        said = field_glb.disagreements(tiny, self.COLLISION)
        self.assertTrue(any('units' in line for line in said), said)

    def test_a_check_with_too_little_to_go_on_says_so_rather_than_passing(self):
        parts = [gltf.Part('Nothing Named', ['Field', 'Nothing Named'], box((0, 0, 0), 0.05), None)]
        said = field_glb.disagreements(parts, self.COLLISION)
        self.assertEqual(1, len(said))
        self.assertIn('could not judge', said[0])

if __name__ == '__main__':
    unittest.main()
