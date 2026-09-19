import json
import os
import re
import unittest

import step_to_field as field

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SIM_PLACEMENT_JAVA = os.path.join(
    REPO, 'TeamCode', 'src', 'test', 'java', 'org', 'firstinspires', 'ftc', 'teamcode', 'sim',
    'SimPlacement.java')


def box(bottom, top, side=4):
    vertices = [(0, 0, bottom), (side, 0, bottom), (side, side, bottom), (0, side, bottom),
                (0, 0, top), (side, 0, top), (side, side, top), (0, side, top)]
    faces = [[0, 1, 2, 3], [7, 6, 5, 4], [0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]
    return vertices, faces


def piece(x, y, z=1.4, radius=1.4, **extra):
    return dict({'name': 'Pollen <1>', 'kind': 'pollen', 'centre': [x, y, z], 'radius': radius}, **extra)


class ReadingStep(unittest.TestCase):
    def test_an_argument_list_becomes_nested_python_values_with_references_named(self):
        self.assertEqual(
            ['', ('ref', 12), [1.0, 2.0, 3.0], '.T.'],
            field.parse_args("('',#12,(1.,2.,3.),.T.)"))

    def test_a_nested_argument_list_keeps_its_nesting(self):
        self.assertEqual(
            [('ref', 1), [('ref', 2), ('ref', 3)]],
            field.parse_args("(#1,(#2,#3))"))

    def test_a_quoted_name_keeps_the_spaces_and_punctuation_inside_it(self):
        self.assertEqual(['am-5853-Blue Hive <1>'], field.parse_args("('am-5853-Blue Hive <1>')"))


class Shapes(unittest.TestCase):
    def test_a_hull_drops_the_points_inside_it_and_winds_counter_clockwise(self):
        self.assertEqual(
            [(0, 0), (2, 0), (2, 2), (0, 2)],
            field.hull2d([(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)]))

    def test_points_on_one_line_hull_to_their_two_ends(self):
        self.assertEqual([(0, 0), (2, 0)], field.hull2d([(0, 0), (1, 0), (2, 0)]))

    def test_simplify_drops_a_corner_that_is_not_one(self):
        self.assertEqual(
            [(0, 0), (2, 0), (2, 2), (0, 2)],
            field.simplify([(0, 0), (1, 0), (2, 0), (2, 2), (0, 2)]))

    def test_simplify_keeps_a_triangle_whatever_its_corners_are_like(self):
        self.assertEqual(3, len(field.simplify([(0, 0), (1, 0.001), (2, 0)])))


class WhatTheRobotRunsInto(unittest.TestCase):
    def test_a_part_standing_on_the_floor_has_the_footprint_it_stands_on(self):
        vertices, faces = box(0, 10)

        self.assertEqual([(0, 0), (4, 0), (4, 4), (0, 4)],
                         field.footprint_below(vertices, faces, 5))

    def test_a_part_that_hangs_entirely_above_the_height_asked_about_has_no_footprint(self):
        vertices, faces = box(6, 10)

        self.assertIsNone(field.footprint_below(vertices, faces, 2))

    def test_a_part_the_height_cuts_through_is_measured_where_it_is_cut(self):
        vertices, faces = box(0, 10)

        self.assertEqual([(0, 0), (4, 0), (4, 4), (0, 4)],
                         field.footprint_below(vertices, faces, 10))


class WhichPiecesTheSimulatorRolls(unittest.TestCase):
    OBSTACLES = [{'name': 'leg', 'footprint': [[0, 0], [2, 0], [2, 2], [0, 2]]}]

    def loose(self, p):
        return field.is_loose(p, 141, self.OBSTACLES)

    def test_a_pollen_resting_on_the_floor_in_the_open_is_loose(self):
        self.assertTrue(self.loose(piece(20, 20)))

    def test_a_pollen_standing_where_an_obstacle_stands_is_not_loose(self):
        self.assertFalse(self.loose(piece(1, 1)))

    def test_a_piece_a_cell_or_a_flower_holds_is_theirs_and_not_loose(self):
        self.assertFalse(self.loose(piece(20, 20, cell='Blue Goal')))
        self.assertFalse(self.loose(piece(20, 20, flower='Flower Assembly <1>')))

    def test_a_piece_off_the_floor_is_not_loose(self):
        self.assertFalse(self.loose(piece(20, 20, z=40)))

    def test_a_piece_outside_the_walls_is_not_loose(self):
        self.assertFalse(self.loose(piece(80, 20)))


class TheFieldFrame(unittest.TestCase):
    def test_the_cad_frame_becomes_road_runners(self):
        self.assertEqual((-3, -1, 2), field.to_field((1, 2, 3)))

    def test_a_parts_catalogue_number_is_not_part_of_its_name(self):
        self.assertEqual('Blue Hive <1>', field.clean('am-5853-Blue Hive <1>'))
        self.assertEqual('Hive Goal Top Skin', field.clean('am-5869: Hive Goal Top Skin'))


class TheModelTheReadersNeed(unittest.TestCase):
    def model(self, **changes):
        whole = {
            'source': 'field.step',
            'size': 141.17,
            'wallHeight': 12.0,
            'elements': [{'group': 'Frame <1>', 'name': 'Leg', 'vertices': [], 'faces': []}],
            'obstacles': [{'name': 'Frame <1> / Leg <1>', 'footprint': [[0, 0], [1, 0], [1, 1]],
                           'clears': 0.0, 'stands': 20.0}],
            'hives': [{'name': 'Blue Hive <1>', 'alliance': 'Blue', 'tilt': 30.0}],
            'flowers': [{'name': 'Flower Assembly <1>', 'axis': [60, 60], 'bore': 3.0}],
            'pieces': [piece(20, 20, loose=True)],
            'tape': [],
        }
        whole.update(changes)
        return whole

    def test_a_whole_model_is_accepted(self):
        self.assertEqual(self.model(), field.checked(self.model()))

    def test_a_model_missing_a_key_a_reader_needs_names_that_key(self):
        for key in ('size', 'wallHeight', 'elements', 'obstacles', 'hives', 'flowers', 'pieces', 'tape'):
            missing = self.model()
            del missing[key]
            with self.assertRaises(ValueError) as refused:
                field.checked(missing)
            self.assertIn(key, str(refused.exception))

    def test_an_obstacle_without_a_footprint_that_encloses_anything_is_refused(self):
        thin = self.model(obstacles=[{'name': 'Frame <1> / Leg <1>', 'footprint': [[0, 0], [1, 0]],
                                      'clears': 0.0, 'stands': 20.0}])

        with self.assertRaises(ValueError) as refused:
            field.checked(thin)

        self.assertIn('footprint', str(refused.exception))

    def test_an_obstacle_that_does_not_say_how_high_it_stands_is_refused(self):
        for key in ('clears', 'stands'):
            obstacle = {'name': 'Frame <1> / Leg <1>', 'footprint': [[0, 0], [1, 0], [1, 1]],
                        'clears': 0.0, 'stands': 20.0}
            del obstacle[key]
            with self.assertRaises(ValueError) as refused:
                field.checked(self.model(obstacles=[obstacle]))
            self.assertIn(key, str(refused.exception))

    def test_a_piece_that_does_not_say_where_it_is_or_how_big_is_refused(self):
        for key in ('centre', 'radius', 'kind', 'loose'):
            one = piece(20, 20, loose=True)
            del one[key]
            with self.assertRaises(ValueError) as refused:
                field.checked(self.model(pieces=[one]))
            self.assertIn(key, str(refused.exception))

    def test_a_field_nothing_could_be_placed_on_is_refused(self):
        with self.assertRaises(ValueError) as refused:
            field.checked(self.model(size=0))

        self.assertIn('size', str(refused.exception))


class TheModelThatIsCommitted(unittest.TestCase):
    def test_the_field_the_simulator_loads_is_one_this_writer_would_accept(self):
        with open(field.OUT) as handle:
            self.assertIsNotNone(field.checked(json.load(handle)))

    def test_the_robot_height_obstacles_are_measured_against_is_the_one_the_simulator_places_with(self):
        source = open(SIM_PLACEMENT_JAVA).read()
        found = re.search(r'ROBOT_SIZE_IN\s*=\s*([0-9.]+)', source)

        self.assertIsNotNone(found, SIM_PLACEMENT_JAVA + ' no longer names ROBOT_SIZE_IN')
        self.assertEqual(float(found.group(1)), float(field.ROBOT_HEIGHT_IN))


if __name__ == '__main__':
    unittest.main()
