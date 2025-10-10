

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
- diff-drive system for robot
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
- sdf format [tree](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_navsat)
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

## URDF
- for urdf testing use `urdf_tutorial` package
```
ros2 launch urdf_tutorial display.launch.py model:=<udrf file path>
```
- `<udrf file path>` have to be absolute path for wsl
```
ros2 launch urdf_tutorial display.launch.py model:=/mnt/d/coding/urdf_ws/src/rover_description/urdf/urdf_wheel.urdf
```
- always confirm the tf tree
```
ros2 run tf2_tools view_frames
```


## Rtab-map
- launch files according to camera sensors [github](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch)
- rtabmap [github](https://github.com/introlab/rtabmap_ros?tab=readme-ov-file#installation)
  - [demos](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos) 

- Data not received error
  - changed the diff-drive fixed-frame from odom -> map
  - commands i tried
  - from github
  ```
  ros2 launch rtabmap_launch rtabmap.launch.py     rtabmap_args:="--delete_db_on_start"     rgb_topic:=/camera/image     depth_topic:=/camera/depth_image     camera_info_topic:=/camera/camera_info     frame_id:=camera_sensor     use_sim_time:=true     approx_sync:=true qos:=2 rviz:=true queue_size:=30
  ```
  - from gpt
  ```
   ros2 launch rtabmap_launch rtabmap.launch.py   rgb_topic:=/camera/image   depth_topic:=/camera/depth_image   camera_info_topic:=/camera/camera_info   frame_id:=camera_sensor   use_sim_time:=true   visual_odometry:=true   approx_sync:=true   qos_image:=1 qos_camera_info:=1 qos_odom:=1
  ```
  - checked the time-stamp header of briged camera & depth_camera data. 
  - tried with direct point cloud input.
  ```
  ros2 launch rtabmap_launch rtabmap.launch.py    rgb_topic:=/camera/image    camera_info_topic:=/camera/camera_info    scan_cloud_topic:=/camera/images/points    frame_id:=camera_sensor    use_sim_time:=true    visual_odometry:=true
  ```
  - with this they look for `/camera/depth_registered/image_raw` 
  - not even rtab_viz  starting
  - another gpt
  ```
   ros2 launch rtabmap_launch rtabmap.launch.py    rgbd_sync:=false    scan_cloud_topic:=/camera/images/points
  ```
  - my initial command
  ```
  ros2 launch rtabmap_launch rtabmap.launch.py    rgb_topic:=/camera/image    depth_topic:=/camera/depth_image    camera_info_topic:=/camera/camera_info    frame_id:=camera_sensor    use_sim_time:=true    visual_odometry:=true
  ```
- corrected command
```
ros2 launch rtabmap_launch rtabmap.launch.py     rtabmap_args:="--delete_db_on_start"     rgb_topic:=/camera/image     depth_topic:=/camera/depth_image     camera_info_topic:=/camera/images/camera_info     frame_id:=camera_sensor     use_sim_time:=true     approx_sync:=true qos:=2 rviz:=true queue_size:=30
```

- lets run with turtle bot first .

## turtlebot
- official [docs](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node)


## Elevation map
- cuda based from leggedrobotics. [link](https://github.com/leggedrobotics/elevation_mapping_cupy). 
  - works in ros2 humble
- anybotics [link](https://github.com/ANYbotics/elevation_mapping/tree/master)
  - no mention of ros2 installation in the readme. but branch exists

## moveit
- binary installation [link](https://moveit.ai/install-moveit2/binary/)

## IMU tools
- imu tool packagee have 2 fusion algo for imu. it provides 4 function

| Node                   | Main Use                    | Lightweight?        | Magnetic Support |
| ---------------------- | --------------------------- | ------------------- | ---------------- |
| `complementary_filter` | Fast, simple orientation    | ✅                   | Optional         |
| `imu_filter_madgwick`  | More accurate fusion        | ⚙️ Slightly heavier | ✅                |
| `imu_transformer`      | Adjust frame rotation       | ✅                   | —                |
| `imu_to_tf`            | Broadcast orientation as TF | ✅                   | —                |

- imu_filter_madgwick
```
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args -r /imu/data_raw:=/imu/raw -r /imu/data:=/imu/madgwick
```
- imu_complementary_filter
```
ros2 run imu_complementary_filter complementary_filter_node \
  --ros-args \
  -r /imu/data_raw:=/imu \
  -r /imu/mag:=/magnet
```

### Testng
- IMU orientation
```
ros2 run rqt_plot rqt_plot /imu/orientation/x:y:z:w
```
- angular velocity
```
ros2 run rqt_plot rqt_plot /imu/angular_velocity/x:y:z
```
- magnetometer
```
ros2 run rqt_plot rqt_plot /magnet/magnetic_field/x:y:z
```

## ros wiki
- wiki.ros -> is for ros1
- index.ros.org -> is for ros2

but i think `wiki.ros` describes better than `index.ros`. ros wiki clearly shows what i need to pass and what will bw the output topics. 


## rqt_plot
- a simple matplot for debugging. A more advance plot option would be `PlotJuggler` 
```
ros2 run rqt_plot rqt_plot /topic
```
- or if i need to a specific field 
```
ros2 run rqt_plot rqt_plot /imu/linear_acceleration/x:y:z
```
