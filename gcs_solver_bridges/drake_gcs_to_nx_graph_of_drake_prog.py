import gcs_solver
import networkx as nx
import numpy as np
import pydrake.geometry.optimization
from matplotlib.patches import FancyArrowPatch
from pydrake.all import (
    AffineBall,
    GraphOfConvexSets,
    Hyperellipsoid,
    MathematicalProgram,
    Point,
    Solve,
    VPolytope,
)

from gcs_solver_bridges.drake_prog_nx_graph_to_gcs_solver_gcs import (
    drake_prog_nx_graph_to_gcs_solver_gcs,
)


def drake_gcs_to_nx_graph_of_drake_prog(gcs: GraphOfConvexSets) -> nx.DiGraph:
    """
    Convert a Drake GraphOfConvexSets to a NetworkX directed graph where each node
    contains the corresponding MathematicalProgram.

    Args:
        gcs: A Drake GraphOfConvexSets instance.

    Returns:
        A NetworkX directed graph with nodes containing MathematicalProgram instances.
    """
    nx_graph = nx.DiGraph()

    # Add nodes with their corresponding MathematicalProgram
    for v in gcs.Vertices():
        prog = MathematicalProgram()
        prog.AddDecisionVariables(v.x())
        v.set().AddPointInSetConstraints(prog, v.x())
        for cost in v.GetCosts({GraphOfConvexSets.Transcription.kRelaxation}):
            prog.AddCost(cost)
        for constraint in v.GetConstraints(
            {GraphOfConvexSets.Transcription.kRelaxation}
        ):
            prog.AddConstraint(constraint)
        nx_graph.add_node(v.id(), prog=prog, name=v.name())

    for e in gcs.Edges():
        prog = MathematicalProgram()
        prog.AddDecisionVariables(e.xu())
        prog.AddDecisionVariables(e.xv())
        for cost in e.GetCosts({GraphOfConvexSets.Transcription.kRelaxation}):
            vars = cost.variables()
            prog.AddDecisionVariables(vars)
            prog.AddCost(cost)
        for constraint in e.GetConstraints(
            {GraphOfConvexSets.Transcription.kRelaxation}
        ):
            vars = constraint.variables()
            prog.AddDecisionVariables(vars)
            prog.AddConstraint(constraint)
        nx_graph.add_edge(e.u().id(), e.v().id(), prog=prog, name=e.name())

    return nx_graph


def drake_gcs_to_gcs_solver_gcs(
    drake_gcs: pydrake.geometry.optimization.GraphOfConvexSets,
):
    nx_gcs = drake_gcs_to_nx_graph_of_drake_prog(drake_gcs)
    gcs_solver_gcs, gcs_solver_to_nx_gcs = drake_prog_nx_graph_to_gcs_solver_gcs(nx_gcs)
    return gcs_solver_gcs, gcs_solver_to_nx_gcs, nx_gcs


# Taken from underactuated notes on GCS
def Plot2dGraphOfConvexSets(gcs, ax):
    # Generate points on the unit circle for plotting ellipsoids
    theta = np.linspace(0, 2 * np.pi, 100)
    circle_points = np.vstack((np.cos(theta), np.sin(theta)))

    for v in gcs.Vertices():
        if isinstance(v.set(), Point):
            ax.plot(v.set().x()[0], v.set().x()[1], "k", marker="o", markersize=5)
            ax.text(
                v.set().x()[0],
                v.set().x()[1] + 0.1,
                v.name(),
                horizontalalignment="center",
            )
        elif isinstance(v.set(), VPolytope):
            ax.fill(
                v.set().vertices()[0, :].T,
                v.set().vertices()[1, :].T,
                "lightgrey",
                edgecolor="k",
            )
            ax.text(
                np.mean(v.set().vertices()[0, :]),
                np.mean(v.set().vertices()[1, :]),
                v.name(),
                horizontalalignment="center",
                verticalalignment="center",
            )
        elif isinstance(v.set(), Hyperellipsoid):
            aball = AffineBall(v.set())
            vertices = aball.B() @ circle_points + aball.center().reshape((2, 1))
            ax.fill(vertices[0, :].T, vertices[1, :].T, "lightgrey", edgecolor="k")
            ax.text(
                v.set().center()[0],
                v.set().center()[1],
                v.name(),
                horizontalalignment="center",
                verticalalignment="center",
            )

    for e in gcs.Edges():
        # Solve a small program to draw the edges
        prog = MathematicalProgram()
        prog.AddDecisionVariables(e.xu())
        e.u().set().AddPointInSetConstraints(prog, e.xu())
        prog.AddDecisionVariables(e.xv())
        e.v().set().AddPointInSetConstraints(prog, e.xv())
        cost = prog.NewContinuousVariables(1, "cost")[0]
        prog.AddLorentzConeConstraint(np.concatenate(([cost], e.xu() - e.xv())))
        prog.AddLinearCost(cost)

        result = Solve(prog)
        assert result.is_success()
        ax.add_patch(
            FancyArrowPatch(
                result.GetSolution(e.xu()),
                result.GetSolution(e.xv()),
                arrowstyle="->",
                mutation_scale=20,
                color="k",
            )
        )


def toy_example():
    gcs = pydrake.geometry.optimization.GraphOfConvexSets()

    source = gcs.AddVertex(Point([0, 0]), "source")
    vertices = np.array([[1, 1, 3, 3], [0, -2, -2, -1]])
    p1 = gcs.AddVertex(VPolytope(vertices), "p1")
    vertices = np.array([[4, 5, 3, 2], [-2, -4, -4, -3]])
    p2 = gcs.AddVertex(VPolytope(vertices), "p2")
    vertices = np.array([[2, 1, 2, 4, 4], [2, 3, 4, 4, 3]])
    p3 = gcs.AddVertex(VPolytope(vertices), "p3")
    e1 = gcs.AddVertex(Hyperellipsoid(np.eye(2), [4, 1]), "e1")
    e2 = gcs.AddVertex(Hyperellipsoid(np.diag([0.5, 1]), [7, -2]), "e2")
    vertices = np.array([[5, 7, 6], [4, 4, 3]])
    p4 = gcs.AddVertex(VPolytope(vertices), "p4")
    vertices = np.array([[7, 8, 9, 8], [2, 2, 3, 4]])
    p5 = gcs.AddVertex(VPolytope(vertices), "p5")
    target = gcs.AddVertex(Point([9, 0]), "target")

    gcs.AddEdge(source, p1)
    gcs.AddEdge(source, p2)
    gcs.AddEdge(source, p3)
    gcs.AddEdge(p1, e2)
    gcs.AddEdge(p2, p3)
    gcs.AddEdge(p2, e1)
    gcs.AddEdge(p2, e2)
    gcs.AddEdge(p3, p2)  # removing this changes the asymptotic behavior.
    gcs.AddEdge(p3, e1)
    gcs.AddEdge(p3, p4)
    gcs.AddEdge(e1, e2)
    gcs.AddEdge(e1, p4)
    gcs.AddEdge(e1, p5)
    gcs.AddEdge(e2, e1)
    gcs.AddEdge(e2, p5)
    gcs.AddEdge(e2, target)
    gcs.AddEdge(p4, p3)
    gcs.AddEdge(p4, e2)
    gcs.AddEdge(p4, p5)
    gcs.AddEdge(p4, target)
    gcs.AddEdge(p5, e1)
    gcs.AddEdge(p5, target)

    # |xu - xv|₂²
    for e in gcs.Edges():
        diff = e.xu() - e.xv()
        e.AddCost(diff.dot(diff))
    return gcs, source, target

    # First solve the convex relaxation
    options = GraphOfConvexSetsOptions()
    options.convex_relaxation = True
    options.preprocessing = False
    result = gcs.SolveShortestPath(source, target, options)
    assert result.is_success()
    print("")
    print(
        f"Solution lower bound (from the relaxation): \t\t{result.get_optimal_cost()}"
    )

    # Now solve it again, with rounding enabled (to find a feasible solution)
    options.max_rounded_paths = 5
    result = gcs.SolveShortestPath(source, target, options)
    assert result.is_success()
    print(
        f"Solution upper bound (from a feasible solution): \t{result.get_optimal_cost()}"
    )
    # If the lower bound and upper bound are equal, then the solution obtained from the
    # relaxation is optimal.

    fig, ax = plt.subplots()
    Plot2dGraphOfConvexSets(gcs, ax)

    path = gcs.GetSolutionPath(source, target, result)
    print("Shortest path: ", end="")
    for e in path:
        vertices = np.vstack((result.GetSolution(e.xu()), result.GetSolution(e.xv())))
        ax.plot(
            vertices[:, 0],
            vertices[:, 1],
            "darkred",
            linewidth=2,
            linestyle="--",
            marker="o",
            markersize=5,
        )

    ax.set_aspect("equal")
    ax.axis("off")


if __name__ == "__main__":
    import faulthandler
    import os
    import signal

    # write faulthandler output to a stable file for post-mortem inspection
    _log_path = os.path.expanduser("~/gcs_solver_segfault.log")
    try:
        _log_file = open(_log_path, "w")
        # Enable faulthandler for all threads (this will print on fatal signals like SIGSEGV)
        faulthandler.enable(file=_log_file, all_threads=True)
        # Register a safe user signal for on-demand trace dumps (optional).
        # Don't attempt to register SIGSEGV/SIGABRT which cannot be registered.
        try:
            faulthandler.register(signal.SIGUSR1, file=_log_file, all_threads=True)
        except Exception:
            # If registration fails (platform-specific), continue — enable() is already set.
            pass
    except Exception:
        # fallback to stderr if opening file fails
        faulthandler.enable(all_threads=True)
    # ...existing code...
    drake_gcs, drake_source, drake_target = toy_example()
    drake_options = pydrake.geometry.optimization.GraphOfConvexSetsOptions()
    drake_options.convex_relaxation = True
    drake_options.preprocessing = False

    drake_result = drake_gcs.SolveShortestPath(
        drake_source, drake_target, drake_options
    )

    nx_gcs = drake_gcs_to_nx_graph_of_drake_prog(drake_gcs)
    nx_source = drake_source.id()
    nx_target = drake_target.id()

    gcs_solver_gcs, gcs_solver_to_nx_gcs = drake_prog_nx_graph_to_gcs_solver_gcs(nx_gcs)
    nx_gcs_to_gcs_solver = {v: k for k, v in gcs_solver_to_nx_gcs.items()}

    import pathlib

    serialize_path = pathlib.Path(
        "/home/amice/Documents/coding_projects/gcs_solver_project/cpp/gcs_solver/test/data/test_gcs.pkl",
    )
    with open(serialize_path, "wb") as f:
        f.write(gcs_solver_gcs.Serialize())
    with open(serialize_path, "rb") as f:
        gcs2 = gcs_solver.GraphOfConvexSets.Deserialize(f.read())

    gcs_solver_source = gcs_solver_gcs.GetVertexById(nx_gcs_to_gcs_solver[nx_source])
    gcs_solver_target = gcs_solver_gcs.GetVertexById(nx_gcs_to_gcs_solver[nx_target])

    for v in gcs_solver_gcs.Vertices():
        print(f"Vertex {v.name()} with id {v.id()}")
        info = v.info()
        print(f" {info.program().A().as_dense()=}")
        print(f" {info.program().b()=}")
        print(f" {info.program().c()=}")
        print(f" {info.program().cones()=}")
        print()
    for v in gcs_solver_gcs.Edges():
        print(f"Edge {v.name()} with id {v.id()}")
        info = v.info()
        print(f" {info.program().A().as_dense()=}")
        print(f" {info.program().b()=}")
        print(f" {info.program().c()=}")
        print()
    from gcs_solver.admm_solver import (
        AdmmSolver,
        AdmmSolverConfigurationOptions,
        AdmmSolverInstrumented,
    )

    config_options = AdmmSolverConfigurationOptions(
        vertex_subspace_solver_type=AdmmSolverConfigurationOptions.VertexSubspaceSolverType.kStein,
        edge_subspace_solver_type=AdmmSolverConfigurationOptions.EdgeSubspaceSolverType.kQdldl,
        qp_solver_type=AdmmSolverConfigurationOptions.QpSolverType.kOsqp,
        constructor_verbosity=AdmmSolverConfigurationOptions.ConstructorVerbosity.kSilent,
    )
    admm_solver = AdmmSolver(
        gcs=gcs_solver_gcs,
        source=gcs_solver_source,
        target=gcs_solver_target,
        config_options=config_options,
    )
    admm_options = gcs_solver.admm_solver.AdmmSolverOptions(
        max_iters=10000,
        eps_abs=1e-4,
        eps_rel=1e-4,
        verbosity=gcs_solver.admm_solver.AdmmSolverOptions.Verbosity.kInfo,
    )
    gcs_solver_result = admm_solver.Solve(admm_options)
    print(gcs_solver_result)
    print(f"Drake GCS optimal cost: {drake_result.get_optimal_cost()}")
    pass
