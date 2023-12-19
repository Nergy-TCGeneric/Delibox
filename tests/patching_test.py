from unittest import TestCase
from mapper import mapper
import numpy as np

TESTING_VALUE = 64
CELL_DEFAULT_VALUE = 128


class PatchingTest(TestCase):
    def test_if_extended_to_right_or_bottom(self):
        global_mapper = mapper.GlobalMapper((3, 3))
        # This point makes minX and minY to 0, respectively
        global_mapper.update_observer_pos(mapper.Point(3, 3))
        global_mapper._occupancy_grid.content.fill(TESTING_VALUE)

        submap = mapper.Map((9, 9))
        bbox = global_mapper._calculate_resized_bbox(submap)
        global_mapper._try_resize_grid_by_submap(submap)

        self.assertTrue(bbox[0] == 0, "Patching offset x is not 0.")
        self.assertTrue(bbox[2] == 0, "Patching offset y is not 0.")

        expected = np.ones((9, 9)) * CELL_DEFAULT_VALUE
        expected[0:3, 0:3] = TESTING_VALUE

        is_equal = np.array_equal(expected, global_mapper._occupancy_grid.content)
        self.assertTrue(is_equal, "Map mismatch.")

    def test_if_extended_to_left(self):
        global_mapper = mapper.GlobalMapper((3, 3))
        global_mapper.update_observer_pos(mapper.Point(1, 3))
        global_mapper._occupancy_grid.content.fill(TESTING_VALUE)

        submap = mapper.Map((9, 9))
        bbox = global_mapper._calculate_resized_bbox(submap)
        global_mapper._try_resize_grid_by_submap(submap)

        # At this point, the patching offset must be (2, 0).
        self.assertEqual(abs(bbox[0]), 2, "Patching offset x mismatch.")

        expected = np.ones((9, 9)) * CELL_DEFAULT_VALUE
        expected[0:3, 2:5] = TESTING_VALUE

        is_equal = np.array_equal(expected, global_mapper._occupancy_grid.content)
        self.assertTrue(is_equal, "Map mismatch.")

    def test_if_extended_to_top(self):
        global_mapper = mapper.GlobalMapper((3, 3))
        global_mapper.update_observer_pos(mapper.Point(3, 1))
        global_mapper._occupancy_grid.content.fill(TESTING_VALUE)

        submap = mapper.Map((9, 9))
        bbox = global_mapper._calculate_resized_bbox(submap)
        global_mapper._try_resize_grid_by_submap(submap)

        self.assertEqual(abs(bbox[2]), 2, "Patching offset y mismatch.")

        expected = np.ones((9, 9)) * CELL_DEFAULT_VALUE
        expected[2:5, 0:3] = TESTING_VALUE

        is_equal = np.array_equal(expected, global_mapper._occupancy_grid.content)
        self.assertTrue(is_equal, "Map mismatch.")
