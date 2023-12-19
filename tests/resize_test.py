from unittest import TestCase
from mapper import mapper


class ResizingTest(TestCase):
    def test_if_submap_is_overlapped_by_grid_when_drawing_point_is_0(self):
        # The observer point is (0, 0), therefore the drawing point is centered on grid.
        global_mapper = mapper.GlobalMapper((50, 50))
        stub_submap = mapper.Map((25, 25))
        bbox = global_mapper._calculate_resized_bbox(stub_submap)
        self.assertEqual(
            bbox,
            (0, 50, 0, 50),
            "Size changed even though the submap is smaller than grid",
        )

    def test_if_submap_is_not_overlapped_on_left_when_drawing_point_is_towarded_to_left_side(
        self,
    ):
        global_mapper = mapper.GlobalMapper((50, 50))
        global_mapper.update_observer_pos(mapper.Point(-25, 0))
        stub_submap = mapper.Map((25, 25))
        bbox = global_mapper._calculate_resized_bbox(stub_submap)

        # Drawing point is at (0, 25), stub_submap is 25x25, rounding toward infinity
        # will make the updated bbox from (0, 50, 0, 50) to (-13, 50, 0, 50).
        self.assertEqual(
            bbox,
            (-13, 50, 0, 50),
            "bbox not changed (on left)",
        )

    def test_if_submap_is_not_overlapped_on_right_when_drawing_point_is_towarded_to_right_side(
        self,
    ):
        global_mapper = mapper.GlobalMapper((50, 50))
        global_mapper.update_observer_pos(mapper.Point(25, 0))
        stub_submap = mapper.Map((25, 25))
        bbox = global_mapper._calculate_resized_bbox(stub_submap)
        self.assertEqual(
            bbox,
            (0, 63, 0, 50),
            "bbox not changed (on right)",
        )

    def test_if_submap_is_not_overlapped_on_top_when_drawing_point_is_0(self):
        global_mapper = mapper.GlobalMapper((50, 50))
        global_mapper.update_observer_pos(mapper.Point(0, -25))
        stub_submap = mapper.Map((25, 25))
        bbox = global_mapper._calculate_resized_bbox(stub_submap)
        self.assertEqual(
            bbox,
            (0, 50, -13, 50),
            "bbox not changed (on top)",
        )

    def test_if_submap_is_not_overlapped_on_bottom_when_drawing_point_is_0(self):
        global_mapper = mapper.GlobalMapper((50, 50))
        global_mapper.update_observer_pos(mapper.Point(0, 25))
        stub_submap = mapper.Map((25, 25))
        bbox = global_mapper._calculate_resized_bbox(stub_submap)
        self.assertEqual(
            bbox,
            (0, 50, 0, 63),
            "bbox not changed (on bottom)",
        )

    def test_if_drawing_point_is_outside_of_grid_while_submap_itself_is_smaller_than_grid(
        self,
    ):
        global_mapper = mapper.GlobalMapper((50, 50))
        global_mapper.update_observer_pos(mapper.Point(50, 50))
        stub_submap = mapper.Map((25, 25))
        bbox = global_mapper._calculate_resized_bbox(stub_submap)

        # Drawing point is at (75, 75).
        self.assertEqual(
            bbox,
            (0, 88, 0, 88),
            "bbox not changed (outside of the grid)",
        )

    def test_if_submap_is_bigger_than_grid_even_if_drawing_point_is_zero(self):
        global_mapper = mapper.GlobalMapper((50, 50))
        stub_submap = mapper.Map((200, 200))
        bbox = global_mapper._calculate_resized_bbox(stub_submap)
        self.assertEqual(
            bbox,
            (-75, 125, -75, 125),
            "bbox not changed (submap bigger than grid)",
        )
