

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

