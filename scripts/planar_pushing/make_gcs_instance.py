from pathlib import Path
import pathlib
from planning_through_contact.experiments.utils import get_default_experiment_plans
from planning_through_contact.experiments.utils import (
    get_default_plan_config,
    get_default_solver_params,
)
from planning_through_contact.planning.planar.planar_pushing_planner import (
    PlanarPushingPlanner,
)
from planning_through_contact.visualize.planar_pushing import (
    compare_trajs,
    make_traj_figure,
    plot_forces,
    visualize_planar_pushing_start_and_goal,
    visualize_planar_pushing_trajectory,
)
from planning_through_contact.geometry.planar.planar_pushing_path import (
    PlanarPushingPath,
)
from pydrake.geometry.optimization import GraphOfConvexSets


def save_relaxed_and_rounded_path(path: PlanarPushingPath, output_folder: Path) -> None:
    traj_relaxed = path.to_traj()
    traj_rounded = path.to_traj(rounded=True)

    make_traj_figure(
        traj_relaxed,
        filename=f"{output_folder}/relaxed_traj",
        split_on_mode_type=True,
        show_workspace=False,
    )

    if traj_rounded is not None:
        make_traj_figure(
            traj_rounded,
            filename=f"{output_folder}/rounded_traj",
            split_on_mode_type=True,
            show_workspace=False,
        )

        compare_trajs(
            traj_relaxed,
            traj_rounded,
            traj_a_legend="relaxed",
            traj_b_legend="rounded",
            filename=f"{output_folder}/comparison",
        )

        visualize_planar_pushing_trajectory(
            traj_relaxed,  # type: ignore
            save=True,
            filename=f"{output_folder}/relaxed_traj",
            visualize_knot_points=False,
        )

        visualize_planar_pushing_trajectory(
            traj_rounded,  # type: ignore
            save=True,
            filename=f"{output_folder}/rounded_traj",
            visualize_knot_points=False,
        )


def create_gcs_instance(
    solve_problem: bool = True,
) -> tuple[GraphOfConvexSets, GraphOfConvexSets.Vertex, GraphOfConvexSets.Vertex]:
    slider_type = "sugar_box"
    pusher_radius = 0.015

    config = get_default_plan_config(
        slider_type=slider_type,
        pusher_radius=pusher_radius,
    )

    seed = 1
    num_boundary_conditions = 50
    idx_to_plan_for = 2  # change this if you want to try different boundary conditions
    start_and_target = get_default_experiment_plans(
        seed, num_boundary_conditions, config
    )[idx_to_plan_for]

    output_dir = Path("OUTPUTS")
    output_dir.mkdir(exist_ok=True)

    DEBUG = True
    if DEBUG:
        visualize_planar_pushing_start_and_goal(
            config.dynamics_config.slider.geometry,
            config.dynamics_config.pusher_radius,
            start_and_target,
            save=True,
            filename=f"{output_dir}/start_and_goal",
        )

    planner = PlanarPushingPlanner(config)
    planner.config.start_and_goal = start_and_target
    planner.formulate_problem()

    gcs_instance = planner.gcs
    source = planner.source.vertex
    target = planner.target.vertex
    solve_problem = False
    if solve_problem:
        solver_params = get_default_solver_params(debug=DEBUG)
        path = planner.plan_path(solver_params)

        # We may get infeasible
        if path is not None:
            save_relaxed_and_rounded_path(path, output_dir)

    return gcs_instance, source, target


def save_gcs(
    gcs,
    s,
    t,
    output_dir,
    mosek_solve_times=None,
    mosek_num_iters=None,
    mosek_status=None,
    scs_solve_times=None,
    scs_num_iters=None,
    scs_status=None,
    source_parser=None,
):
    output_dir = pathlib.Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_dir / "gcs.pybin", "wb") as f:
        f.write(gcs.Serialize())
    with open(output_dir / "gcs.pybin", "rb") as f:
        import gcs_solver

        gcs2 = gcs_solver.GraphOfConvexSets.Deserialize(f.read())
        pass
    info_text = (
        f"source_parser: {source_parser}\n"
        f"s_id: {s.id().get_value()}\n"
        f's_name: "{s.name()}"\n'
        f"t_id: {t.id().get_value()}\n"
        f't_name: "{t.name()}"\n'
        f"num_vertices: {gcs.num_vertices()}\n"
        f"num_edges: {gcs.num_edges()}\n"
        f"mosek_solve_time_ms: {mosek_solve_times}\n"
        f"mosek_num_iters: {mosek_num_iters}\n"
        f"mosek_status: {mosek_status}\n"
        f"scs_solve_time_ms: {scs_solve_times}\n"
        f"scs_num_iters: {scs_num_iters}\n"
        f"scs_status: {scs_status}\n"
    )
    info_path = output_dir / "info.yaml"

    info_path.write_text(info_text, encoding="utf-8")


if __name__ == "__main__":
    from gcs_solver import GraphOfConvexSets
    from pydrake.all import GraphOfConvexSetsOptions, CommonSolverOption
    import pydrake.solvers
    from gcs_solver_bridges.drake_gcs_to_nx_graph_of_drake_prog import (
        drake_gcs_to_gcs_solver_gcs,
    )

    drake_gcs, drake_source, drake_target = create_gcs_instance()
    gcs_solver_gcs, gcs_solver_to_nx_gcs, nx_gcs = drake_gcs_to_gcs_solver_gcs(
        drake_gcs,
    )
    nx_gcs_to_gcs_solver = {v: k for k, v in gcs_solver_to_nx_gcs.items()}
    nx_source = drake_source.id()
    nx_target = drake_target.id()
    gcs_solver_source = gcs_solver_gcs.GetVertexById(nx_gcs_to_gcs_solver[nx_source])
    gcs_solver_target = gcs_solver_gcs.GetVertexById(nx_gcs_to_gcs_solver[nx_target])
    mosek_solve_times = []
    mosek_num_iters = []
    mosek_status = None
    scs_solve_times = []
    scs_num_iters = []
    scs_status = None
    num_trials = 0
    output_dir = Path("tmp")
    for _ in range(num_trials):
        if True:
            print(
                f"Solving from {drake_source.name()} to {drake_target.name()} with commercial solver"
            )
            options = GraphOfConvexSetsOptions()
            options.convex_relaxation = True
            options.preprocessing = False
            options.max_rounded_paths = 0
            options.solver = pydrake.solvers.MosekSolver()
            options.solver_options.SetOption(CommonSolverOption.kPrintToConsole, True)

            options.solver_options.SetOption(
                pydrake.solvers.MosekSolver().id(),
                "MSK_IPAR_PRESOLVE_USE",
                0,
            )

            result = drake_gcs.SolveShortestPath(
                drake_source, drake_target, options=options
            )
            mosek_details = result.get_solver_details()
            mosek_solve_times.append(mosek_details.optimizer_time * 1000.0)
            mosek_status = result.get_solution_result()
            mosek_num_iters.append(None)  # drake doesn't return this

            options.solver_options.SetOption(
                pydrake.solvers.MosekSolver().id(),
                "MSK_IPAR_PRESOLVE_USE",
                1,
            )
            result2 = drake_gcs.SolveShortestPath(
                drake_source, drake_target, options=options
            )
            details2 = result2.get_solver_details()

            options.solver = pydrake.solvers.ScsSolver()
            result = drake_gcs.SolveShortestPath(
                drake_source, drake_target, options=options
            )
            scs_details = result.get_solver_details()
            scs_solve_times.append(scs_details.scs_solve_time)
            scs_status = result.get_solution_result()
            scs_num_iters.append(scs_details.iter)
        this_output_dir = output_dir
        save_gcs(
            gcs_solver_gcs,
            gcs_solver_source,
            gcs_solver_target,
            this_output_dir,
            source_parser="drake",
            scs_solve_times=scs_solve_times,
            mosek_solve_times=mosek_solve_times,
            mosek_num_iters=mosek_num_iters,
            scs_num_iters=scs_num_iters,
            mosek_status=mosek_status,
            scs_status=scs_status,
        )
    save_gcs(
        gcs_solver_gcs,
        gcs_solver_source,
        gcs_solver_target,
        output_dir,
        source_parser="drake",
        scs_solve_times=scs_solve_times,
        mosek_solve_times=mosek_solve_times,
        mosek_num_iters=mosek_num_iters,
        scs_num_iters=scs_num_iters,
        mosek_status=mosek_status,
        scs_status=scs_status,
    )
    pass
