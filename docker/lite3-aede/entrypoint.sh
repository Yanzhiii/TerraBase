#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/terrabase/terra_base_ws/install/setup.bash

exec "$@"
