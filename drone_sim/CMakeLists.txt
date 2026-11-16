cmake_minimum_required(VERSION 3.8)
project(drone_sim)
find_package(ament_cmake REQUIRED)
# Tools that run as ROS nodes are installed so launch files can start them.
# models/ and worlds/ are used straight from the source tree through
# GZ_SIM_RESOURCE_PATH (set by ros_env), so editing them needs no rebuild.
install(PROGRAMS tools/truth_relay.py tools/imu_drift_sim.py tools/stream_gate.py tools/pose_mangler.py tools/range_relay.py tools/make_maze.py
                 tools/stub_world.py tools/stub_core.py tools/make_fixtures.py tools/fault_planner.py
                 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
ament_package()
