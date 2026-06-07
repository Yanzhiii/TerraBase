#!/usr/bin/env python3
"""Run upstream go2_robot_sdk after preparing TerraBase install-space shims."""

from pathlib import Path
from shutil import copyfile

from ament_index_python.packages import get_package_share_directory


def _ensure_aioice_marker() -> None:
    sdk_share = Path(get_package_share_directory('go2_robot_sdk'))
    bridge_share = Path(get_package_share_directory('go2_bridge'))

    target = sdk_share / 'external_lib' / 'aioice' / '__init__.py'
    if target.exists():
        return

    source = bridge_share / 'compat' / 'aioice' / '__init__.py'
    target.parent.mkdir(parents=True, exist_ok=True)
    copyfile(source, target)


def main():
    _ensure_aioice_marker()

    from go2_robot_sdk.main import main as sdk_main
    sdk_main()


if __name__ == '__main__':
    main()
