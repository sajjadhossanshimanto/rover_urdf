

- gazebo harmonic [docs](https://gazebosim.org/docs/harmonic/install_ubuntu/)
## Errors
- nothing

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


## TODO
- add `base_footprint` link
    - currently the axis zero is located in one of the wheel
    - need to move the origine to `base_footprint` straight bellow the `base_link`

