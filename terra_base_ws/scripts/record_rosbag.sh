#!/bin/bash
#
# Record a rosbag for offline AEDE testing.
#
# Usage:
#   ./record_rosbag.sh [bag_name]
#
# Records:
#   /odom              - robot odometry
#   /point_cloud2      - L1 LiDAR point cloud (raw, odom frame)
#   /joint_states      - joint positions
#   /state_estimation  - AEDE-compatible odometry (from bridge)
#   /registered_scan   - AEDE-compatible point cloud (map frame)
#   /terrain_map       - AEDE terrain map
#   /path              - AEDE selected path
#   /cmd_vel           - AEDE command
#   /cmd_vel_out       - clamped Go2 command
#   /tf /tf_static     - all transforms

BAG_NAME="${1:-go2_test_$(date +%Y%m%d_%H%M%S)}"
BAG_DIR="${HOME}/go2_rosbags"
mkdir -p "${BAG_DIR}"

echo "=========================================="
echo " Recording rosbag: ${BAG_NAME}"
echo " Output: ${BAG_DIR}/${BAG_NAME}"
echo "=========================================="
echo ""
echo " Topics being recorded:"
echo "   /odom"
echo "   /point_cloud2"
echo "   /joint_states"
echo "   /state_estimation"
echo "   /registered_scan"
echo "   /terrain_map"
echo "   /path"
echo "   /cmd_vel"
echo "   /cmd_vel_out"
echo "   /tf"
echo "   /tf_static"
echo ""
echo " Press Ctrl+C to stop recording."
echo ""

ros2 bag record \
  -o "${BAG_DIR}/${BAG_NAME}" \
  /odom \
  /point_cloud2 \
  /joint_states \
  /state_estimation \
  /registered_scan \
  /terrain_map \
  /path \
  /cmd_vel \
  /cmd_vel_out \
  /tf \
  /tf_static
