# drone_bringup

Starts everything and holds the flight-controller settings.

* `launch/sim.launch.py`: the ROS side of a simulation profile (`gps`, `truth`, `slam`): Gazebo bridge, robot state,
  ground truth, Cartographer, the bridge, the simulated MTF-01P. Started by `scripts/sim_up.sh`
* `config/gz_bridge.yaml` (Gazebo topics into ROS), `bag_topics.txt` (what each run records), `drone.rviz`
* `params/ardupilot/`: `sitl_base.parm`, `sitl_gps.parm` (GPS profile), `sitl_extnav.parm` (flying on our position: the
  final B settings; lines marked RE-SET must be measured on the real drone), `sitl_flow.parm` (optional flow fusion, B7)
* `params/ardupilot/lua/extnav_fallback.lua`: runs on the flight controller: optical-flow fallback when our position
  stops, back after 1 s of good positions, LAND after 8 s
