import unittest

import gcs_solver
import networkx as nx
import numpy as np
from pydrake.solvers import MathematicalProgram, Solve

from .. import drake_prog_nx_graph_to_gcs_solver_gcs, drake_prog_to_gcs_solver_conic


class TestDrakeProgToGcsSolverConic(unittest.TestCase):
    def check_conversion_via_round_trip(self, prog: MathematicalProgram):
        gcs_conic_prog, drake_conic_form = (
            drake_prog_to_gcs_solver_conic.drake_prog_to_gcs_solver_conic(prog)
        )

        checking_prog = MathematicalProgram()
        x = checking_prog.NewContinuousVariables(gcs_conic_prog.x_size(), "x")
        cone_start = 0
        for cone in gcs_conic_prog.cones():
            cone_dim = cone.dim()
            cur_A = gcs_conic_prog.A()[cone_start : cone_start + cone_dim, :]
            cur_b = gcs_conic_prog.b()[cone_start : cone_start + cone_dim]
            if type(cone) == gcs_solver.ZeroCone:
                checking_prog.AddLinearEqualityConstraint(cur_A, cur_b, x)
            elif type(cone) == gcs_solver.NonnegativeOrthant:
                checking_prog.AddLinearConstraint(
                    cur_A, cur_b, np.inf * np.ones(cone_dim), x
                )
            elif type(cone) == gcs_solver.LorentzCone:
                checking_prog.AddLorentzConeConstraint(cur_A, -cur_b, x)
            else:
                raise ValueError(f"Unsupported cone type: {type(cone)}")
            cone_start += cone_dim
        checking_prog.AddLinearCost(gcs_conic_prog.c(), gcs_conic_prog.d(), x)

        original_result = Solve(prog)
        checking_result = Solve(checking_prog)
        self.assertEqual(
            original_result.get_solution_result(), checking_result.get_solution_result()
        )
        if checking_result.is_success():
            self.assertAlmostEqual(
                original_result.get_optimal_cost(),
                checking_result.get_optimal_cost(),
                places=3,
            )
            np.testing.assert_allclose(
                original_result.get_x_val(),
                checking_result.get_x_val()[: prog.num_vars()],
                rtol=1e-5,
                atol=1e-5,
            )

    def test_lp(self):
        # From Drake's LinearPogram0 in solvers/test/linear_program_example.cc
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddLinearCost(2 * x[0] + x[1] + 4)

        prog.AddLinearConstraint(-x[0] + x[1] <= 1)
        prog.AddLinearConstraint(2 <= x[0] + x[1])
        prog.AddLinearConstraint(x[0] - 2 * x[1] <= 4)
        prog.AddLinearConstraint(x[1] >= 2)
        prog.AddLinearConstraint(x[0] >= 0)
        self.check_conversion_via_round_trip(prog)

    def test_socp(self):
        # From Drake's MinimimalDistanceFromSphereProblem
        prog = MathematicalProgram()
        n = 3
        x = prog.NewContinuousVariables(n, "x")
        pt = np.array([1, 2, 3])
        center = np.zeros(n)
        radius = 0.5
        prog.AddQuadraticErrorCost(1, pt, x)
        lorentz_cone_expr = np.hstack([[radius], x - center])
        prog.AddLorentzConeConstraint(lorentz_cone_expr)

        self.check_conversion_via_round_trip(prog)


class TestDrakeProgNxGraphToGcsSolverGcs(unittest.TestCase):
    def drake_point_in_circle_program(self, center, radius):
        prog = MathematicalProgram()
        x = prog.NewContinuousVariables(2, "x")
        prog.AddQuadraticCost((x - center).dot(x - center))
        prog.AddQuadraticAsRotatedLorentzConeConstraint(
            2 * np.eye(2), -2 * center, center.dot(center) - radius * radius, x
        )
        return prog

    def make_edge_cost_conic_prog(self, prog1, prog2):
        xv1 = prog1.decision_variables()
        xv2 = prog2.decision_variables()
        prog = MathematicalProgram()
        prog.AddDecisionVariables(xv1)
        prog.AddDecisionVariables(xv2)
        prog.AddQuadraticCost((xv1 - xv2).dot(xv1 - xv2))
        return prog

    def test_can_convert_simple(self):
        drake_graph = nx.DiGraph()
        drake_graph.add_node(
            "u",
            name="u",
            prog=self.drake_point_in_circle_program(np.array([0, 0]), 1.0),
        )
        drake_graph.add_node(
            "v",
            prog=self.drake_point_in_circle_program(np.array([1, 2]), 1.0),
        )

        edge_prog = self.make_edge_cost_conic_prog(
            drake_graph.nodes["u"]["prog"], drake_graph.nodes["v"]["prog"]
        )
        drake_graph.add_edge("u", "v", prog=edge_prog)

        gcs = (
            drake_prog_nx_graph_to_gcs_solver_gcs.drake_prog_nx_graph_to_gcs_solver_gcs(
                drake_graph
            )
        )

        self.assertEqual(len(gcs.Vertices()), 2)
        self.assertEqual(len(gcs.Edges()), 1)
        self.assertEqual(gcs.Vertices()[0].name(), "u")
        self.assertEqual(gcs.Vertices()[1].name(), "v")
        self.assertEqual(gcs.Edges()[0].name(), "u->v")
