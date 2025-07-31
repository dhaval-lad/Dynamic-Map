# dynamic_map

![dynamic_map_gif](resource/dynamic_map.gif)

## Description
The `dynamic_map` package is responsible for using laser scan data and the robot's odometry to create a dynamic local map and costmap around the robot. The maps move with the robot and are updated simultaneously.

## Dependencies
- [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)
- [ROS2-Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- A robot with a 2D LiDAR sensor and odometry information. Recomended: [GrassHopper Gazebo](https://github.com/dhaval-lad/grasshopper_gazebo.git)

## Installation Instructions
1. First, clone this repository inside the `src` folder of your ROS 2 workspace (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws/src
    git clone https://github.com/dhaval-lad/Dynamic-Map.git
    ```
2. Next, build your ROS 2 workspace to install the package (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select dynamic_map
    ```
## Usage Instructions
To verify that everything is working:
1. Run any Gazebo world in which the robot has a 2D LiDAR plugin available.
2. Change the topic names of `/scan` and `/odom` in `dynamic_map.py` to match the topic names in your environment.
3. Rebuild the package:
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select dynamic_map
    ```
4. As the map is being published on the `map` frame, which is not usually present in the robot, you need to run a static transform publisher between the `map` and `odom` frames:
    ```sh
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
    ```
5. Run the following command in the terminal to get `/local_map` and `/costmap`:
    ```sh
    ros2 run dynamic_map dynamic_map
    ```
6. To visualize the data, run `rviz2`, set `map` as the global frame, and add the respective `/local_map` and `/costmap` topics.

## Configuration
You can configure various parameters in `dynamic_map.py`:
- Change the resolution of the map by modifying `self.resolution`.
- Adjust the map height and width using `self.map_height` and `self.map_width`.
- Offset the map in the X-axis of the robot by changing `self.map_height_offset`.
- Offset the map in the Y-axis of the robot by changing `self.map_width_offset`.
- If there is a Yaw offset between base_link and laser, then add that to `self.baselink_to_laser_yaw_offset`.
- Change the update rate with `self.update_rate`.
- Set the expansion size of the costmap using `self.expansion_size`.

## Nodes
- `dynamic_map`: This node uses `/scan` and `/odom` to generate and publish `/local_map` and `/costmap`.

## Topics
- **Subscribed Topics**:
  - `/scan`
  - `/odom`
- **Published Topics**:
  - `/local_map`
  - `/costmap`

## License
This project is licensed under the Apache License, Version 2.0, January 2004. See [http://www.apache.org/licenses/](http://www.apache.org/licenses/) for more details.
