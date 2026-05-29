# TurtleBot4 Known Issues and References

This note collects external issues and discussions that are relevant to the
B002 TurtleBot4 setup. The problems seen in this project are not from a single
source: part of the behavior comes from Wi-Fi/DDS/network stability, part from
hardware services such as Create 3, RPLidar and OAK-D, and part from how the
TurtleBot4 and Nav2 packages are launched and sequenced.

The links below are references used while debugging the B002 map, localization,
Nav2 startup, dock/undock behavior, simulation and mission routines.

## Network, DDS and connectivity

These references are relevant when topics appear and disappear, services hang,
nodes can see each other but data does not flow, or the robot loses the base /
Raspberry Pi link.

- [TurtleBot4 #659 - Random connectivity failures, shutdown issues, base-Raspberry link loss, controller failures and unstable behavior](https://github.com/turtlebot/turtlebot4/issues/659)
- [TurtleBot4 #22 - Robot connectivity issues](https://github.com/turtlebot/turtlebot4/issues/22)
- [TurtleBot4 #598 - Localization or Nav2 on multiple TB4s at the same time](https://github.com/turtlebot/turtlebot4/issues/598)
- [Create 3 docs discussion #392 - ROS 2 communication failures](https://github.com/iRobotEducation/create3_docs/discussions/392)
- [Nav2 #3033 - Fast-DDS service reliability sometimes hangs lifecycle manager](https://github.com/ros-navigation/navigation2/issues/3033)
- [Unix StackExchange - NetworkManager Wi-Fi secrets issue](https://unix.stackexchange.com/questions/420640/unable-to-connect-to-any-wifi-with-networkmanager-due-to-error-secrets-were-req)
- [Robotics StackExchange - Communication between different ROS 2 distributions](https://robotics.stackexchange.com/questions/94344/can-nodes-from-different-ros-2-distributions-communicate-compatibly)

Project notes:

- The TurtleBot4 side runs ROS 2 Jazzy with FastDDS on `ROS_DOMAIN_ID=0`.
- The MyCobot side runs ROS 2 Galactic separately. Do not rely on a full ROS
  bridge between Jazzy and Galactic for the arm; the integration layer only
  mirrors the joint state needed for visualization.
- Avoid keeping old RViz, Nav2, localization or mission-manager processes alive
  while retesting. Duplicate nodes and stale DDS participants can make failures
  look random.

## Navigation, localization and Nav2 startup

These references match issues where localization, lifecycle startup or Nav2
controllers are not ready every time, or goals abort even though the map and
waypoints look correct.

- [TurtleBot4 #466 - Navigation stack startup problems](https://github.com/turtlebot/turtlebot4/issues/466)
- [TurtleBot4 #341 - TurtleBot4 cannot 2D navigate](https://github.com/turtlebot/turtlebot4/issues/341)
- [TurtleBot4 #543 - Inconsistent navigation on existing maps](https://github.com/turtlebot/turtlebot4/issues/543#issuecomment-2705593025)
- [TurtleBot4 #574 - Localization and navigation do not work every time](https://github.com/turtlebot/turtlebot4/issues/574)
- [TurtleBot4 #635 - Navigation not working: controller_server process has died](https://github.com/turtlebot/turtlebot4/issues/635)
- [Nav2 #2917 - Lifecycle manager will not startup nodes](https://github.com/ros-navigation/navigation2/issues/2917)
- [Nav2 #3033 - Fast-DDS service reliability sometimes hangs lifecycle manager](https://github.com/ros-navigation/navigation2/issues/3033)

Project notes:

- Always set the initial pose in RViz after launching localization with the B002
  map and before expecting autonomous routines to work.
- Wait for Nav2 lifecycle nodes to become active before running
  `mission_manager.py`.
- The mission code waits for fresh `/scan`, `/odom` and `map -> base_link` TF
  before sending navigation goals. This avoids sending a goal while the lidar or
  AMCL is still recovering after undock.

## LaserScan, TF and stale timestamp failures

These references are relevant when logs contain messages like:

- `the timestamp on the message is earlier than all the data in the transform cache`
- `Lookup would require extrapolation into the past`
- `Robot pose is not available`
- `[scan]: Latest source and current collision monitor node timestamps differ`

References:

- [ROS Answers - LIDAR timestamp earlier than transform cache](https://answers.ros.org/question/393581/)
- [ROS Answers comment thread for the same TF/cache issue](https://answers.ros.org/question/393581/?comment=395859#post-id-395859)
- [TurtleBot4 #574 - Localization and navigation do not work every time](https://github.com/turtlebot/turtlebot4/issues/574)
- [TurtleBot4 #635 - controller_server process has died](https://github.com/turtlebot/turtlebot4/issues/635)

Useful checks:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /amcl_pose --once
```

If `/scan` is stale after undock, restart or recover the TurtleBot4 bringup
before starting a mission. In the B002 routine the mission manager now waits for
fresh sensor input, but the underlying lidar stream still needs to become stable.

## Mapping, SLAM and saved maps

These references are useful when generating maps, loading existing maps or
combining SLAM/navigation flows.

- [TurtleBot4 #457 - Generate a map tutorial error](https://github.com/turtlebot/turtlebot4/issues/457)
- [TurtleBot4 #242 - Navigating while mapping tutorial request](https://github.com/turtlebot/turtlebot4/issues/242)
- [TurtleBot4 navigation tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)

Project notes:

- B002 uses the saved map at `maps/B002_map.yaml`.
- For physical navigation, run localization against that map and set the initial
  pose before launching missions.

## Dock, undock and simulation gaps

These references are relevant when comparing the real robot with Gazebo or when
dock/undock behavior differs between physical and simulated setups.

- [TurtleBot4 simulator #38 - Simulator dock/undock](https://github.com/turtlebot/turtlebot4_simulator/issues/38)
- [TurtleBot4 simulator #47 - Custom world file in simulation](https://github.com/turtlebot/turtlebot4_simulator/issues/47)

Project notes:

- Physical dock/undock uses the Create 3 actions `/dock` and `/undock`.
- After undock, the RPLidar may need extra time to restart or provide fresh
  scans. The mission code intentionally waits before sending the first Nav2 goal.
- The simulation path is useful for development, but it is not a perfect model
  of the real B002 room, dock behavior or sensor recovery timing.

## Obstacle avoidance and hardware behavior

These references are relevant when local costmaps, obstacle avoidance or robot
performance differ from expectations.

- [TurtleBot4 #141 - Obstacle avoidance and performance issues](https://github.com/turtlebot/turtlebot4/issues/141)
- [TurtleBot4 #659 - Connectivity and unstable hardware behavior](https://github.com/turtlebot/turtlebot4/issues/659)

Project notes:

- The B002 3D visualization is not the collision model used by Nav2. Nav2 uses
  the 2D map and live sensor data.
- Objects visible in the 3D room should match the obstacles the robot actually
  mapped at lidar height; otherwise the visualization can look realistic but
  mislead debugging.

## Practical reset sequence

When a run becomes inconsistent, stop local ROS tools and start from a clean
graph:

```bash
pkill -f "mission_manager.py" || true
pkill -f "turtlebot4_navigation localization.launch.py" || true
pkill -f "turtlebot4_navigation nav2.launch.py" || true
pkill -f "turtlebot4_viz view_navigation.launch.py" || true
pkill -f "rviz2" || true
ros2 daemon stop
ros2 daemon start
```

Then start localization, set the initial pose, start Nav2, open RViz, and only
then run the mission manager.
