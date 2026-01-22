import os
import pathlib
import sys
from pathlib import Path

# Add parent gcs_examples directory to path first, before any other imports
# This submodule contains its own gcs_solver_bridges/ which would shadow the parent's
# Need the parent's gcs_solver_bridges for run_generators to work
GCS_EXAMPLES_DIR = Path(__file__).parent.parent
# Remove current directory from path if present (it gets added automatically)
SUBMODULE_DIR = str(Path(__file__).parent)
if SUBMODULE_DIR in sys.path:
    sys.path.remove(SUBMODULE_DIR)
# Insert parent at the very beginning
if str(GCS_EXAMPLES_DIR) in sys.path:
    sys.path.remove(str(GCS_EXAMPLES_DIR))
sys.path.insert(0, str(GCS_EXAMPLES_DIR))

import hydra
import matplotlib.pyplot as plt
from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig, OmegaConf

import dropbox
from gcs_examples import run_generators

# Now add submodule back for planar_pushing imports
if SUBMODULE_DIR not in sys.path:
    sys.path.append(SUBMODULE_DIR)

# Register resolvers for config interpolation
OmegaConf.register_new_resolver(
    "dropbox_gcs_examples_path", lambda: dropbox.DROPBOX_GCS_EXAMPLES_PATH, replace=True
)


@hydra.main(version_base=None, config_path="conf", config_name="debug")
def main(cfg: DictConfig) -> None:
    # Get the hydra output directory (where .hydra config is automatically written)
    hydra_cfg = HydraConfig.get()
    output_dir = pathlib.Path(hydra_cfg.runtime.output_dir)
    gcs_path = output_dir / "gcs.pybin"
    if os.path.exists(gcs_path) and not cfg.force_remake:
        print(f"Example at {gcs_path} exists. Skipping")
        return

    # Generate the GCS from planar pushing parameters
    params = cfg.planar_params

    # Import and call make_planar_pushing_gcs function
    from planar_pushing import make_planar_pushing_gcs

    try:
        drake_gcs, drake_source, drake_target, env_info, planner = make_planar_pushing_gcs(
            slider_type=params.slider_type,
            pusher_radius=params.pusher_radius,
            seed=params.seed,
            num_boundary_conditions=params.num_boundary_conditions,
            idx_to_plan_for=params.idx_to_plan_for,
            output_dir=output_dir,
            save_visualization=False,
        )

        print(f"Created planar pushing GCS: num_vertices={env_info['num_vertices']}, num_edges={env_info['num_edges']}")

        # Use run_generators.run_drake to solve and save
        run_generators.run_drake(
            drake_gcs, drake_source, drake_target, output_dir, num_trials=cfg.num_trials
        )
    except Exception as e:
        import traceback
        print(f"ERROR: Failed to generate GCS for slider_type={params.slider_type}, seed={params.seed}, idx={params.idx_to_plan_for}")
        print(f"Exception type: {type(e).__name__}: {e}")
        print("\nFull traceback:")
        traceback.print_exc()
        return


if __name__ == "__main__":
    main()
