import unittest
from w9_pathfinding.envs import Grid
from w9_pathfinding.mapf import ReservationTable, ResumableSpaceTimeDijkstra


class TestResumableSpaceTimeDijkstra(unittest.TestCase):
    """
    pytest tests/mapf/test_resumable_space_time_dijkstra.py::TestResumableSpaceTimeDijkstra
    """

    def test_grid(self):
        grid = Grid([[1, 1, 1], [-1, -1, 1], [1, 1, 1]])
        rs = ResumableSpaceTimeDijkstra(grid, (0, 0))

        inf = float("inf")

        for n, ans in [((0, 0), 0), ((0, 1), inf), ((0, 2), 6)]:
            self.assertEqual(rs.distance(n), ans)

        rs.start_node = (2, 1)
        for n, ans in [((0, 0), 3), ((0, 1), inf), ((0, 2), 3)]:
            self.assertEqual(rs.distance(n), ans)

    def test_reservation_table(self):
        grid = Grid([[1, 1, 1], [1, -1, 1], [1, 1, 1]], edge_collision=True)
        rt = ReservationTable(grid)
        rt.add_path([(0, 2), (0, 2), (0, 1), (0, 0)], reserve_destination=True)

        rs = ResumableSpaceTimeDijkstra(grid, (0, 0), rt)

        path = rs.find_path((1, 2))
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2)])

        path = rs.find_path((0, 2))
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2)])

        path = rs.find_path((0, 1))
        self.assertEqual(path, [(0, 0), (0, 1)])

    def test_time(self):
        weights = [[1, 1, 0.9], [1, -1, 1], [0.2, 0.1, 0.2]]
        grid = Grid(weights=weights, pause_weights=weights)
        rt = ReservationTable(grid)

        rs = ResumableSpaceTimeDijkstra(
            grid, start_node=(0, 0), reservation_table=rt, time_horizon=10
        )

        inf = float("inf")
        for time, distance_answer, path_answer in [
            (0, inf, []),
            (1, inf, []),
            (2, inf, []),
            (3, 2.9, [(0, 0), (1, 0), (2, 0), (2, 1)]),
            (4, 3.8, [(0, 0), (1, 0), (2, 0), (2, 0), (2, 1)]),
            (5, 2.5, [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 1)]),
            (6, 2.6, [(0, 0), (0, 1), (0, 2), (1, 2), (1, 2), (2, 2), (2, 1)]),
            (None, 2.5, [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 1)]),
        ]:
            path = rs.find_path((2, 1), time)
            self.assertEqual(path, path_answer)

            distance = rs.distance((2, 1), time)
            self.assertAlmostEqual(distance, distance_answer, delta=0.001)

    def test_with_reserved_goal(self):
        weights = [[1, 0.5, 1, 1]]
        grid = Grid(weights=weights, pause_weights=weights)
        rt = ReservationTable(grid)
        rt.add_path([(2, 0), (2, 0), (2, 0), (2, 0), (3, 0)], reserve_destination=True)

        rs = ResumableSpaceTimeDijkstra(
            grid, start_node=(0, 0), reservation_table=rt, time_horizon=10
        )

        inf = float("inf")
        for time, distance_answer, path_answer in [
            (None, 2.5, [(0, 0), (1, 0), (1, 0), (1, 0), (2, 0)]),
            (1, inf, []),
            (2, inf, []),
            (3, inf, []),
            (4, 2.5, [(0, 0), (1, 0), (1, 0), (1, 0), (2, 0)]),
            (5, 3.0, [(0, 0), (1, 0), (1, 0), (1, 0), (1, 0), (2, 0)]),
        ]:
            path = rs.find_path((2, 0), time)
            self.assertEqual(path, path_answer)

            distance = rs.distance((2, 0), time)
            self.assertAlmostEqual(distance, distance_answer, delta=0.001)
