import unittest
from w9_pathfinding import mapf
from w9_pathfinding.envs import Graph
from tests.mapf.test_grid_mapf import MAPF_ALGORITHMS, COMPLETE_ALGORITHMS, check_paths


class TestGraphMAPF(unittest.TestCase):
    """
    pytest tests/mapf/test_graph_mapf.py::TestGraphMAPF
    """

    def test_1(self):
        graph = Graph(5, edges=[[0, 2], [1, 2], [2, 3], [2, 4]])
        starts = [0, 1]
        goals = [3, 4]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](graph).mapf(starts, goals, **a.get("params", {}))
                self.assertEqual(len(paths), 0, msg=paths)

    def test_1_with_self_loops(self):
        graph = Graph(5, edges=[[0, 2], [1, 2], [2, 3], [2, 4]])
        graph.add_edges([[0, 0, 0.5], [1, 1, 1]])  # self loops
        starts = [0, 1]
        goals = [3, 4]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](graph).mapf(starts, goals, **a.get("params", {}))
                self.assertEqual(len(paths), 2)

                if a["class"] in (mapf.MultiAgentAStar, mapf.CBS):
                    # Optimal algorithms should find the optimal solution
                    self.assertEqual(paths[0], [0, 0, 2, 3])
                    self.assertEqual(paths[1], [1, 2, 4])
                else:
                    # The others should find some solution
                    self.assertIn(
                        paths, ([[0, 0, 2, 3], [1, 2, 4]], [[0, 2, 3], [1, 1, 2, 4]])
                    )

    def test_2(self):
        graph = Graph(3, edges=[[0, 1], [1, 2], [2, 0]])
        starts = [1, 0]
        goals = [1, 2]

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](graph).mapf(starts, goals, **a.get("params", {}))
                self.assertEqual(len(paths), 2)
                self.assertEqual(paths[0], [1, 2, 0, 1])
                self.assertEqual(paths[1], [0, 1, 2])

    def test_3(self):
        graph = Graph(4, edges=[[0, 1], [1, 2], [2, 3], [3, 0]])
        starts = [1, 2, 3]
        goals = [0, 2, 3]

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](graph).mapf(starts, goals, **a.get("params", {}))
                self.assertEqual(len(paths), 3)
                self.assertEqual(
                    paths, [[1, 2, 3, 0], [2, 3, 0, 1, 2], [3, 0, 1, 2, 3]]
                )
