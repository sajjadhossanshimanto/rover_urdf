

- gazebo harmonic [docs](https://gazebosim.org/docs/harmonic/install_ubuntu/)
## Errors
- gps data is fluctuating in our empty world but not in `spherical_coordinates.sdf`

## ros_gz bridge
- https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
- be carefull about the version and branch
- https://github.com/gazebosim/ros_gz/tree/77522600db37d49a23e349c6e109b08caa621188/ros_gz_bridge -> previous commit that uses `gz`
- ros humble with gazebo hermonic
- https://gazebosim.org/docs/harmonic/ros2_integration/ -> example with `gz`
- https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md -> examples are with `ignition`

- run ros_gz_bridge config 
```
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=src/rover_description/config/gazebo_bridge.yaml
```
## ros controll
- send controll command from ros. to check if bridge is working 
```
ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
```
- teleop controller
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- there is also package available for joystick `teleop_twist_joy`

## Gazebo plugin
- system repo -> https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems
- It's always better to follow version spacefic instruction
- we are using `gazebosim` and version 8+. so branch is `gz-sim8`

### gazebo topic
- list topic
```
gz topic -l
```
- send command to a topic
```
gz topic -t "/model/rover_robo/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
``` 

## gazebo sensors
- sensors [library](https://github.com/gazebosim/gz-sensors)
- stf format [tree](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_navsat)
- predefined world [sdf](https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds)
- add in the world
```
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
```
### Gps
- how code works at [sensor.cc](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/NavSatSensor.cc#L82)
- all attributes [header.hh](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1NavSatSensor.html)
- example world [spherical_coordinates](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/spherical_coordinates.sdf)
- sdf [tree](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_gps)

## TODO
- add `base_footprint` link
    - currently the axis zero is located in one of the wheel
    - need to move the origine to `base_footprint` straight bellow the `base_link`

## Slam toolbox
- slam toolbox works only with 2D lider data
- to use 3D depth data 
  - we may convert into 2D lider data using `pointcloud_to_laserscan` 
  - or for 3D visualisation we use `rtab_map`

- param file for slam. copy it from installed folder
- most important thing to modify are
```
    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping #localization
```
- running the slam toolbox
```
ros2 launch slam_toolbox online_async_launch.py param_file:=src/rover_description/config/mapper_params_online_async.yaml use_sim_tgime:=true
```
- add the `map` topic in rviz. 

### pointcloud_to_laserscan
- intall the package
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-pointcloud-to-laserscan
```
- runing the package 
```
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node
```
- cofigure a yaml file. all param list [ros index](https://index.ros.org/p/pointcloud_to_laserscan/)
```xml
pointcloud_to_laserscan_node:
  ros__parameters:
    target_frame: base_link
    transform_tolerance: 0.01
    min_height: 0.0
    max_height: 1.0
    angle_min: -1.57
    angle_max: 1.57
    angle_increment: 0.0087
    scan_time: 0.033
    range_min: 0.45
    range_max: 10.0
    use_inf: true
    input_pointclud_topic: /camera/images/points
    output_scan_topic: /scan
```
- run with config file (not sure if working)
```
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --params-file ~/your_path/pointcloud_to_laserscan.yaml
```
- run with just changeing the input topic (haven't confirm if working)
```
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r input:=/camera/images/points
```

### ISSUE
- `/scan` topic is created by empty 
- it is confirmed that point clound is not working

