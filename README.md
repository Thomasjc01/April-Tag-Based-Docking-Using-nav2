# April Tag Docking with Turtlebot Waffle

This project demonstrates autonomous docking of a Turtlebot Waffle towards an April Tag using ROS 2 and the Nav2 stack. The robot navigates in a custom-built environment, utilizing SLAM for mapping, AMCL for localization, and both global and local planners for path planning. Key components include camera image rectification, April Tag detection, and waypoint navigation to achieve precise docking.
Here is a <video src="https://drive.google.com/file/d/1TDNuCQh9E_LkHFtGMoSdu3ceFN-_GrSW/view?usp=sharing" controls></video> demonstrating the docking


## Features

- **Custom World:** The project uses a custom simulation world created in Gazebo.
- **Mapping and Localization:**
  - SLAM Toolbox was used to create a map of the environment.
  - AMCL (Adaptive Monte Carlo Localization) is employed for accurate robot localization.
- **Camera Integration:**
  - The Turtlebot Waffle’s built-in camera plugin provides RGB and depth optical frames.
  - The `image_proc` package is used to rectify camera images, reducing distortion.
- **April Tag Detection:**
  - April Tags are detected using the `apriltag_ros` package.
  - The package publishes transformations (TFs) between the `camera_depth_optical_frame` and `tag_frame`.
- **Navigation:**
  - A\* is used as the global planner for efficient pathfinding.
  - DWA (Dynamic Window Approach) is utilized as the local planner for smooth navigation.
- **Docking:**
  - Nav2’s FollowWaypoints behavior is leveraged to guide the robot towards the April Tag and dock at the station.

## System Requirements

- ROS 2 (tested with Humble or later)
- Gazebo (compatible version for ROS 2)
- Turtlebot3 packages
- SLAM Toolbox
- Nav2 stack
- `image_proc` package
- `apriltag_ros` package

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Thomasjc01/April-Tag-Based-Docking-Using-nav2
   ```
2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Usage

### 1. Launching the Simulation

Start the Gazebo simulation with the custom world:

```bash
ros2 launch april_tag_docking bringup.launch.py
```

### 2. Mapping the Environment

Use the SLAM Toolbox to create a map:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

Save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

### 3. Localization

Run AMCL to localize the robot in the mapped environment:

```bash
ros2 launch my_package world_launch.py
```

### 4. Image Rectification and April Tag Detection

Detect April Tags in the environment and publish their tf:

```bash
ros2 launch my_docking_pkg apriltag_tf_publisher.launch.py
```

### 5. Navigation 

Launch the nav2 Node:

```bash
ros2 launch nav2_pkg nav2_launch.py
```

### 6. Docking

Move towards the april tag and perform docking.
```bash
ros2 run my_docking_pkg dock.py
```

## Workflow

1. **Image Rectification:** The `image_proc` node rectifies the raw camera images, mitigating distortion for accurate detection.
2. **April Tag Detection:** The `apriltag_ros` package identifies tags and publishes their TF relative to the camera’s optical frame.
3. **Transform Calculation:** Using the TF between the map frame and the docking station’s tag frame, the docking station’s coordinates are computed.
4. **Navigation:** Nav2’s FollowWaypoints guides the Turtlebot Waffle to the docking station using the calculated coordinates.


## Results

- The Turtlebot Waffle successfully detects the April Tag, navigates to the docking station, and docks accurately using ROS 2 and Nav2-based waypoint following.
- Dynamic Obstable Avoidance using the Local Planner.
- This approach for docking does not rely on continuous camera data. Once the tf is published, camera data is no longer necessary, making it deployable in environments where camera data is not always available.

## Future Improvements

- Optimize camera calibration for higher precision in April Tag detection.






