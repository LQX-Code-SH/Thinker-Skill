#!/usr/bin/env bash
# 在 manip_local:v4 容器内 source。宿主机不要直接 source。
source /opt/ros/jazzy/setup.bash
if [ -f /opt/xmigcs-venv/bin/activate ]; then
    source /opt/xmigcs-venv/bin/activate
    export PYTHONPATH="/opt/xmigcs-venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi
# SDK（可选）：由 XHUMANOID_SDK_DIR 挂到 /opt/xhumanoid_sdk
if [ -d /opt/xhumanoid_sdk ]; then
    if [ -f /opt/xhumanoid_sdk/install/setup.bash ]; then
        source /opt/xhumanoid_sdk/install/setup.bash
    fi
    export PYTHONPATH="/opt/xhumanoid_sdk:/opt/xhumanoid_sdk/python:${PYTHONPATH:-}"
fi
export LD_LIBRARY_PATH="/opt/xos/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export FASTRTPS_DEFAULT_PROFILES_FILE="/thinker/sim/robot_sim/config/fastdds/fastdds_shm.xml"
