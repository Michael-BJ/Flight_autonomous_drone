#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
cd "$(dirname "$0")"
mkdir -p mission_logs
i=11
for sc in tl_nominal tl_jump tl_baro_lost tl_kill fm_nominal tl_bigoffset; do
  ROS_DOMAIN_ID=$i RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout 300 /usr/bin/python3 offline_run_mission.py $sc > mission_logs/$sc.log 2>&1 &
  i=$((i+1))
done
wait
for sc in tl_nominal tl_jump tl_baro_lost tl_kill fm_nominal tl_bigoffset; do
  echo "== $sc"; grep '^RESULT' mission_logs/$sc.log || tail -3 mission_logs/$sc.log
done
