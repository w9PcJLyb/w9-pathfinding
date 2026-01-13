import unittest
from w9_pathfinding.envs import Grid
from w9_pathfinding.mapf import CBS


class TestCBS(unittest.TestCase):
    """
    pytest tests/mapf/test_cbs.py::TestCBS
    """

    def test_seed(self):
        """
        + -  -  -  -  -  - +
        |    s1    #  #  # |
        | s2       #     g1|
        |                g2|
        |          #     g3|
        |    s3    #  #  # |
        + -  -  -  -  -  - +
        """
        weights = [
            [1, 1, 1, -1, -1, -1],
            [1, 1, 1, -1, 1, 1],
            [1, 1, 1, 1, 1, 1],
            [1, 1, 1, -1, 1, 1],
            [1, 1, 1, -1, -1, -1],
        ]
        grid = Grid(weights=weights, edge_collision=True)
        starts = [(1, 0), (0, 1), (1, 4)]
        goals = [(5, 1), (5, 2), (5, 3)]

        # This problem has more than 100 optimal solutions.
        # Providing a seed should make the solver's behavior reproducible.

        def run(seed):
            cbs = CBS(grid, seed=seed)
            return (
                cbs.mapf(starts, goals, disjoint_splitting=True),
                cbs.mapf(starts, goals, disjoint_splitting=True),
            )

        # Two solvers constructed with the same seed must produce
        # identical sequences of results.
        r11, r12 = run(42)
        r21, r22 = run(42)
        self.assertEqual(r11, r21)
        self.assertEqual(r12, r22)

        # Resetting the seed should restore the initial RNG state.
        cbs = CBS(grid, seed=10)
        r1 = cbs.mapf(starts, goals, disjoint_splitting=True)

        cbs.set_seed(10)
        r2 = cbs.mapf(starts, goals, disjoint_splitting=True)

        self.assertEqual(r1, r2)
