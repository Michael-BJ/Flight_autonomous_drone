Offline tests (no MAVROS, no arming, no hardware). Run with the SYSTEM python
under the ROS env (conda deactivate first):

  source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
  /usr/bin/python3 offline_test_vfh_core.py     # vfh_core == the user's original VFH class; steering signs
  /usr/bin/python3 offline_test_guidance.py     # VFHFlightBaroNode tick-level behaviour (no spin)
  ./offline_run_all_missions.sh                 # fake PX4 + synthetic depth world + real VFH node + real flight node

offline_run_vfh_mission.py <scenario> runs one closed-loop mission and prints a
RESULT json line. Files are named offline_* so `colcon test` / pytest does not
collect them.
