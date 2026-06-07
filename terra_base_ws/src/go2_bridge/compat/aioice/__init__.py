"""Compatibility shim for upstream go2_robot_sdk's expected aioice layout.

The upstream SDK exits during import unless
share/go2_robot_sdk/external_lib/aioice/__init__.py exists. TerraBase installs
this file into that location, then delegates actual imports to the normal
Python environment. Install `aioice` with pip or initialize the upstream
submodule.
"""

from pathlib import Path
import importlib.machinery
import importlib.util
import sys


def _load_real_aioice():
    current_dir = Path(__file__).resolve().parent
    blocked_paths = {current_dir, current_dir.parent}
    search_paths = []

    for path_entry in sys.path:
        try:
            resolved = Path(path_entry).resolve()
        except (OSError, RuntimeError):
            search_paths.append(path_entry)
            continue
        if resolved not in blocked_paths:
            search_paths.append(path_entry)

    spec = importlib.machinery.PathFinder.find_spec(__name__, search_paths)
    if spec is None or spec.loader is None:
        raise ImportError(
            "TerraBase aioice shim could not find a real aioice package. "
            "Run `pip install aioice` or initialize the go2_ros2_sdk submodule."
        )

    module = importlib.util.module_from_spec(spec)
    sys.modules[__name__] = module
    spec.loader.exec_module(module)
    return module


_real_module = _load_real_aioice()
globals().update(_real_module.__dict__)
