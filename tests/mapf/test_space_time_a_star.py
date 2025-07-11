import unittest
from w9_pathfinding.envs import Grid, Graph, DiagonalMovement
from w9_pathfinding.mapf import SpaceTimeAStar, ReservationTable


class TestSpaceTimeAStar(unittest.TestCase):
    """
    pytest tests/mapf/test_space_time_a_star.py::TestSpaceTimeAStar
    """

    def test_with_directed_graph(self):
        graph = Graph(3, edges=[[0, 0], [0, 1], [1, 2]])
        rt = ReservationTable(graph)
        rt.add_path([1, 1, 1])

        a = SpaceTimeAStar(graph)
        path = a.find_path(0, 2, reservation_table=rt)
        self.assertListEqual(path, [0, 0, 0, 1, 2])

    def test_with_directed_weighted_graph(self):
        graph = Graph(5, edges=[[0, 1, 100], [0, 4, 1], [1, 2, 100], [2, 3, 100]])

        a = SpaceTimeAStar(graph)
        path = a.find_path(0, 3)
        self.assertListEqual(path, [0, 1, 2, 3])

    def test_with_grid(self):
        weights = [[1, 1, 1], [1, 1, 1]]
        grid = Grid(weights=weights)
        rt = ReservationTable(grid)
        rt.add_path([(1, 0), (1, 0), (1, 0), (1, 0)])

        a = SpaceTimeAStar(grid)
        path = a.find_path((0, 0), (2, 0), reservation_table=rt)
        self.assertListEqual(path, [(0, 0), (0, 1), (1, 1), (2, 1), (2, 0)])

    def test_3x4_with_two_agents(self):
        """
        + -  -  -  - +
        | s        # |
        |    #     # |
        |       e2 e1|
        + -  -  -  - +
        """
        weights = [[1, 2, 0.9, -1], [1, -1, 0.9, -1], [1, 1, 1, 1]]
        grid = Grid(weights=weights, diagonal_movement=DiagonalMovement.never)

        a = SpaceTimeAStar(grid)

        path1 = a.find_path((0, 0), (3, 2))
        self.assertListEqual(path1, [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (3, 2)])

        rt = ReservationTable(grid)
        rt.add_path(path1)

        grid.pause_weights = 5
        path2 = a.find_path((0, 0), (2, 2), reservation_table=rt)
        self.assertListEqual(
            path2, [(0, 0), (1, 0), (2, 0), (2, 1), (2, 0), (2, 1), (2, 2)]
        )

        grid.pause_weights = 0.1
        path2 = a.find_path((0, 0), (2, 2), reservation_table=rt)
        self.assertListEqual(path2, [(0, 0), (0, 0), (0, 1), (0, 2), (1, 2), (2, 2)])

    def test_search_depth(self):
        grid = Grid(width=5, height=5)
        start, end = (0, 0), (4, 0)

        a = SpaceTimeAStar(grid)
        for d in range(8):
            with self.subTest(f"search_depth={d}"):
                path = a.find_path_with_depth_limit(start, end, search_depth=d)
                path = path[1:]  # ignore start
                self.assertEqual(len(path), min(d, 4))

    def test_search_depth_with_dynamic_obstacles(self):
        grid = Grid(width=4, height=4, edge_collision=True)
        start, end = (0, 0), (3, 0)

        rt = ReservationTable(grid)
        obstacle_path = [(2, 0), (2, 0), (2, 0), (3, 0), (3, 0), (3, 1), (3, 1)]
        rt.add_path(obstacle_path)

        a = SpaceTimeAStar(grid)
        for d in range(8):
            with self.subTest(f"search_depth={d}"):
                path = a.find_path_with_depth_limit(
                    start, end, search_depth=d, reservation_table=rt
                )
                path = path[1:]  # ignore start
                self.assertEqual(len(path), min(d, 5))

    def test_search_depth_2(self):
        """
        + - - - - - - +
        | s x         |
        | # x     #   |
        | x x   #     |
        | x # #     # |
        | x x x   # # |
        |     #     e |
        + - - - - - - +
        """
        weights = [
            [1, 1, 1, 1, 1, 1],
            [-1, 1, 1, 1, -1, 1],
            [1, 1, 1, -1, 1, 1],
            [1, -1, -1, 1, 1, -1],
            [1, 1, 1, 1, -1, -1],
            [1, 1, -1, 1, 1, 1],
        ]
        start, end = (0, 0), (5, 5)

        grid = Grid(weights=weights, diagonal_movement=DiagonalMovement.never)

        a = SpaceTimeAStar(grid)
        path = a.find_path_with_depth_limit(start, end, search_depth=8)
        self.assertEqual(len(path), 9)
        self.assertListEqual(
            path,
            [(0, 0), (1, 0), (1, 1), (1, 2), (0, 2), (0, 3), (0, 4), (1, 4), (2, 4)],
        )

    def test_path_with_exact_length(self):
        """
        + -  -  - +
        | #     # |
        | s     e |
        |         |
        + -  -  - +
        """
        grid = Grid(
            weights=[[-1, 1, -1], [1, 1, 1], [1, 1, 1]],
            pause_weights=[[1, 0.1, 1], [1, 1, 1], [1, 1, 1]],
        )

        a = SpaceTimeAStar(grid)
        path = a.find_path_with_exact_length((0, 1), (2, 1), length=5)

        # The direct path [(0,1), (1,1), (2,1)] has length 3 and cost 2.
        # Extending it naively to length 5 by pausing at (2,1) results in:
        #  - [(0,1), (1,1), (2,1), (2,1), (2,1)] with cost = 5.
        # A better path uses the cheaper pause cost at (1,0), giving:
        #  - [(0,1), (1,1), (1,0), (1,0), (1,1), (2,1)] with cost = 4.1.
        self.assertEqual(path, [(0, 1), (1, 1), (1, 0), (1, 0), (1, 1), (2, 1)])
        self.assertEqual(grid.calculate_cost(path), 4.1)
