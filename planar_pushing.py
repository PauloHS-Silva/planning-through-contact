"""Planar pushing GCS generator (extracted and modified from bernhardpg/planning-through-contact repository)."""

from __future__ import annotations

import sys
from pathlib import Path

# Add the submodule to path for imports
SUBMODULE_DIR = Path(__file__).parent
if str(SUBMODULE_DIR) not in sys.path:
    sys.path.insert(0, str(SUBMODULE_DIR))

from planning_through_contact.experiments.utils import (
    get_default_experiment_plans,
    get_default_plan_config,
)
from planning_through_contact.planning.planar.planar_pushing_planner import (
    PlanarPushingPlanner,
)
from planning_through_contact.visualize.planar_pushing import (
    visualize_planar_pushing_start_and_goal,
)


def make_planar_pushing_gcs(
    slider_type: str = "sugar_box",
    pusher_radius: float = 0.015,
    seed: int = 1,
    num_boundary_conditions: int = 50,
    idx_to_plan_for: int = 0,
    output_dir: Path = None,
    save_visualization: bool = False,
):
    """
    Create a Drake GraphOfConvexSets for planar pushing planning.
    
    Args:
        slider_type: Type of slider object. Options: "box", "sugar_box", "tee", 
                     "convex_4", "convex_5", "triangle"
        pusher_radius: Radius of the pusher (default: 0.015)
        seed: Random seed for generating boundary conditions
        num_boundary_conditions: Number of start/goal pairs to generate
        idx_to_plan_for: Which boundary condition to use (0 to num_boundary_conditions-1)
        output_dir: Directory for saving visualizations (optional)
        save_visualization: Whether to save start/goal visualization
    
    Returns:
        drake_gcs: Drake's GraphOfConvexSets object
        drake_source: Source vertex from Drake GCS
        drake_target: Target vertex from Drake GCS
        env_info: Dictionary with environment information
        planner: The PlanarPushingPlanner object (for plotting)
    """
    # Get configuration
    config = get_default_plan_config(
        slider_type=slider_type,
        pusher_radius=pusher_radius,
    )
    
    # Generate start and target configurations
    start_and_target = get_default_experiment_plans(
        seed, num_boundary_conditions, config
    )[idx_to_plan_for]
    
    # Optionally save visualization
    if save_visualization and output_dir is not None:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        visualize_planar_pushing_start_and_goal(
            config.dynamics_config.slider.geometry,
            config.dynamics_config.pusher_radius,
            start_and_target,
            save=True,
            filename=f"{output_dir}/start_and_goal",
        )
    
    # Create planner and formulate problem
    planner = PlanarPushingPlanner(config)
    planner.config.start_and_goal = start_and_target
    planner.formulate_problem()
    
    # Extract GCS
    drake_gcs = planner.gcs
    drake_source = planner.source.vertex
    drake_target = planner.target.vertex
    
    # Create env_info dict
    env_info = {
        'slider_type': slider_type,
        'pusher_radius': pusher_radius,
        'seed': seed,
        'num_boundary_conditions': num_boundary_conditions,
        'idx_to_plan_for': idx_to_plan_for,
        'num_vertices': drake_gcs.num_vertices(),
        'num_edges': drake_gcs.num_edges(),
    }
    
    return drake_gcs, drake_source, drake_target, env_info, planner
