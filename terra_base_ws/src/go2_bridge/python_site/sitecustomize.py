"""Keep ROS Humble binary Python extensions on their NumPy 1.x ABI.

This module is loaded automatically by Python when its directory is placed on
PYTHONPATH. It preloads Ubuntu's NumPy before the user site-packages directory
can provide NumPy 2.x, then restores the original import path so user-installed
packages such as torch remain available.
"""

import importlib
import sys


_SYSTEM_DIST_PACKAGES = '/usr/lib/python3/dist-packages'

if 'numpy' not in sys.modules:
    sys.path.insert(0, _SYSTEM_DIST_PACKAGES)
    try:
        importlib.import_module('numpy')
    finally:
        try:
            sys.path.remove(_SYSTEM_DIST_PACKAGES)
        except ValueError:
            pass
