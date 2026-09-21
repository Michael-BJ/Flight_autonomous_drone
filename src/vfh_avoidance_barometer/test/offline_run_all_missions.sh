#!/bin/bash
# Closed-loop VFH missions: fake PX4 + synthetic depth world + real nodes.
# Each scenario in its own process / ROS domain, in parallel (~2-3 min).
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
cd "$(dirname "$0")"
mkdir -p mission_logs
SC="open pole_center pole_left pole_right gate wall vfh_lost kill"
i=21
for sc in $SC; do
  ROS_DOMAIN_ID=$i RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout 330 /usr/bin/python3 offline_run_vfh_mission.py $sc > mission_logs/$sc.log 2>&1 &
  i=$((i+1))
done
wait
for sc in $SC; do
  echo "== $sc"; grep '^RESULT' mission_logs/$sc.log || tail -3 mission_logs/$sc.log
done
