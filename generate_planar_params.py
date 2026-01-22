import random
import sys
import tempfile
import yaml
from pathlib import Path
from typing import Any, Dict, List, Optional, Set
import numpy as np


def _fmt(v: Any) -> str:
    """Format values so filenames are readable and stable."""
    if v is None:
        return "None"
    if isinstance(v, bool):
        return "True" if v else "False"
    if isinstance(v, float):
        return f"{v:.3f}".rstrip("0").rstrip(".")
    return str(v)


def generate_filename(config: Dict[str, Any], existing_files: Set[str]) -> str:
    """
    Generate YAML filename with actual values.
    Format:
    {slider_type}_{pusher_radius}_{seed}_{idx}.yaml
    """

    parts = [
        _fmt(config["slider_type"]),
        _fmt(config["pusher_radius"]),
        _fmt(config["seed"]),
        _fmt(config["idx_to_plan_for"]),
    ]

    filename = "_".join(parts) + ".yaml"
    original_seed = config["seed"]

    # Ensure uniqueness by bumping seed if needed
    attempt = 0
    max_attempts = 100
    while filename in existing_files and attempt < max_attempts:
        attempt += 1
        config["seed"] = original_seed + attempt
        parts[2] = _fmt(config["seed"])
        filename = "_".join(parts) + ".yaml"

    if attempt >= max_attempts:
        import time
        config["seed"] = int(time.time())
        parts[2] = _fmt(config["seed"])
        filename = "_".join(parts) + ".yaml"

    return filename


def main(output_dir: Optional[Path] = None) -> List[Path]:
    """
    Generate planar pushing parameter YAML files.
    
    Args:
        output_dir: Directory to write parameter files to. If None, uses system temp.
    
    Returns:
        List of paths to generated YAML files
    """
    COUNT = 150
    SEED = 0
    
    random.seed(SEED)
    np.random.seed(SEED)
    
    # Set output directory, use temp directory if not specified
    if output_dir is None:
        output_dir = Path(tempfile.gettempdir()) / "gcs_examples_temp_planar_params"
    
    # Create output directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Get existing files to avoid duplicates
    existing_files = set(f.name for f in output_dir.glob("*.yaml") if f.is_file())
    
    # Generate config files
    generated_files = []
    
    # Available slider types (from utils.py get_default_plan_config)
    slider_types = ["box", "sugar_box", "tee", "convex_4", "convex_5", "triangle"]
    
    # Pusher radius options
    pusher_radii = [0.01, 0.015, 0.02, 0.025]
    
    # Number of boundary conditions to generate per seed
    num_boundary_conditions = 50
    
    for _ in range(COUNT):
        slider_type = random.choice(slider_types)
        pusher_radius = random.choice(pusher_radii)
        seed = random.randint(0, 100)
        idx_to_plan_for = random.randint(0, num_boundary_conditions - 1)
        
        config = {
            'slider_type': slider_type,
            'pusher_radius': pusher_radius,
            'seed': seed,
            'num_boundary_conditions': num_boundary_conditions,
            'idx_to_plan_for': idx_to_plan_for,
        }
        
        # Generate filename based on all parameter values
        filename = generate_filename(config, existing_files)
        filepath = output_dir / filename
        existing_files.add(filename)  # Add to set to avoid duplicates
        
        # Write config file
        with open(filepath, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=True, allow_unicode=True)
        
        generated_files.append(filepath)
    
    return generated_files


if __name__ == "__main__":
    main()
