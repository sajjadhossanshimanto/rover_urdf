
- gazebo harmonic [docs](https://gazebosim.org/docs/harmonic/install_ubuntu/)

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
gz topic -t "/model/my_robot/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
``` 


## TODO
- add `base_footprint` link
    - currently the axis zero is located in one of the wheel
    - need to move the origine to `base_footprint` straight bellow the `base_link`

