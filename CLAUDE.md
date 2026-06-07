# CLAUDE.md

TerraBase — ROS 2 Humble overlay for Unitree Go2 + ZED 2i + STEPP + AEDE.

## Internal Documentation

- `docs/` is gitignored. Put design decisions, integration notes, troubleshooting records, and agent-shared context there. It stays local and is never pushed.
- `Project Reference.md` is also gitignored — local research planning material.

## Submodule Policy

Third-party dependencies live in `third_party/` as git submodules pointing to forks under `github.com/Yanzhiii`. Each fork carries minimal, documented patches for ROS 2 Humble + real-robot compatibility.

| Submodule | Fork | Patches |
|-----------|------|---------|
| `go2_ros2_sdk` | Yanzhiii/go2_ros2_sdk | cv_bridge lazy-import |
| `autonomous_exploration_development_environment` | Yanzhiii/autonomous_exploration_development_environment | static_transform_publisher Humble syntax |
| `STEPP-Code` | Yanzhiii/STEPP-Code | SLIC viz + zero-range div fix |
| `zed-ros2-wrapper` | stereolabs/zed-ros2-wrapper (upstream) | none |

When making changes to a submodule: commit inside the submodule, push to the fork, then `git add` the submodule in the main repo to record the new pointer.

## Workspace Layout

```
terra_base_ws/src/go2_bridge/    ← TerraBase-owned integration code
terra_base_ws/src/<symlinks>     → third_party submodule packages
third_party/                     ← git submodules (forks)
```

## Key Files

- `terra_base_ws/src/go2_bridge/launch/go2_aede_real.launch.py` — Go2 + AEDE real-robot launch
- `terra_base_ws/src/go2_bridge/launch/go2_zed_stepp_aede.launch.py` — full ZED + STEPP + AEDE stack
- `terra_base_ws/src/go2_bridge/go2_bridge/bridge_node.py` — Go2 ↔ AEDE topic bridge
- `terra_base_ws/src/go2_bridge/go2_bridge/stepp_depth_projector.py` — STEPP cost → PointCloud2 projection
- `terra_base_ws/src/go2_bridge/config/extrinsics.yaml` — ZED mount extrinsics
- `.gitmodules` — submodule URLs and branches
