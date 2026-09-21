Offline tests (no MAVROS, no arming, no hardware). Run with the SYSTEM python
under the ROS env (conda deactivate first):

  source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
  /usr/bin/python3 offline_test_estimator.py           # estimator + closed-loop sim
  ./offline_run_all_missions.sh                         # 6 fake-PX4 missions (~3 min, parallel)
  /usr/bin/python3 ../../fm_deploy_barometer/test/offline_test_recovery.py   # return-home mixin + node

offline_run_mission.py <scenario> runs one fake-PX4 mission and prints a
RESULT json line (scenarios listed in the file). These files are named
offline_* so `colcon test` / pytest does not collect them.
