# TurtleBot3 Path Tracking with Obstacle Avoidance

## Introduction

This project implements a complete navigation pipeline for a differential drive robot (TurtleBot3) using ROS2 Humble. It includes:

- Path smoothing using Cubic Spline interpolation  
- Trajectory generation with time-based sampling  
- Pure Pursuit controller for trajectory tracking  
- Reactive obstacle avoidance using LiDAR scan data  
- Simulation in Gazebo and visualization in RViz  

---

## Core Concepts Explained

### 🔹 1. Path Smoothing Algorithm

When a robot is given a set of waypoints like `[(x0, y0), (x1, y1), ..., (xn, yn)]`, the transitions can be sharp and inefficient.

The robot is given a set of predefined waypoints to follow:

```python
WAYPOINTS = [(0, 0), (1, 0), (2, 2), (3, 0), (4, 2)]
```

![path_comparison](https://github.com/user-attachments/assets/c923be6b-06c8-4d36-9c72-25540e7fc95b)



These waypoints define a zigzag trajectory with sharp turns. If followed directly, the robot would experience jerky motion and potential instability, especially at high speeds or during tight maneuvers.

To solve this, we apply **Cubic Spline Interpolation**:
- Uses arc-length parameterization for uniform sampling
- Fits smooth, continuous curves between waypoints
- Produces a refined and smooth list of intermediate points for the robot to track

📊 _Before and After Visualization_:


### 2. Trajectory Generation

Once we have a smooth path, we generate a **time-parameterized trajectory**:
- Assign velocity profiles (constant by default)
- Sample points along the path and assign timestamps
- Used by the controller to follow the path with timing

Trajectory format:
```python
[(x0, y0, t0), (x1, y1, t1), ..., (xn, yn, tn)]
```

After path smoothing, the next step is to generate a **time-parameterized trajectory** that the controller can follow. This is handled in `trajectory_generator.py`.

Two types of trajectory generation are implemented:


#### A. From Smoothed Path

The function `generate_trajectory()` takes in a list of `(x, y)` points and assigns time values to each based on a fixed velocity:

```python
def generate_trajectory(smooth_path, velocity=0.1):
```

- It calculates the Euclidean distance between consecutive points.
- Computes the time required to travel each segment using `velocity = distance / time`.

This representation is ideal for time-based control.

Example use case: tracking a **smoothed path** generated from cubic splines.


#### B. From Raw Waypoints (Dense Interpolation)

The function `generate_dense_trajectory_from_raw_path()` is used to densify raw waypoints into a smoother linearly interpolated trajectory:

- It performs linear interpolation between each pair of waypoints.
- Each segment is subdivided into `points_per_segment` intermediate points.
- Time is calculated for each point using a constant velocity.
- Useful for trajectory generation **without applying smoothing**.

Example use case: for sharp or jagged paths where spline smoothing is not desired (e.g., precise industrial tasks).


####  Error Handling

Both functions check for valid input:
- If the path has fewer than 2 points, a `ValueError` is raised.
- This ensures no invalid trajectories are passed to the controller.


These generated trajectories are then passed into the Pure Pursuit controller for execution.



###  3. Pure Pursuit Controller

To follow the trajectory generated from the waypoints or smoothed path, the project uses the **Pure Pursuit algorithm**, implemented in `controller.py`.

This controller calculates linear and angular velocity commands that guide the robot toward a point ahead on the path.


#### How It Works

```python
def pure_pursuit_controller(current_pose, path, lookahead=0.3):
```

- **`current_pose`**: The robot’s current location and heading `(x, y, yaw)`
- **`path`**: A list of `(x, y, t)` trajectory points
- **`lookahead`**: Distance ahead on the path the robot tries to “chase”


#### Nearest Point Calculation

```python
def nearest_point_index(path, x, y):
```

- Computes the index of the point on the path **closest** to the current robot position.
- Uses Euclidean distance with `np.hypot()` for robustness.


#### Finding the Goal Point

Once the nearest point is found, the controller searches **forward** along the path to find a goal point that is at least `lookahead` meters ahead.

If no such point exists (i.e., robot has reached the end), the controller stops:
```python
return 0.0, 0.0
```


#### Computing Control Commands

Once a goal point is found:
- The controller computes the angle from the robot to the goal.
- Calculates the **angular error** between the robot's current heading and that goal direction.
- Generates:
  - A **fixed linear velocity** (`v = 0.1`)
  - A **proportional angular velocity** based on angle error: `w = 1.5 * angle_error`

This ensures:
- Smooth curve following
- Stable and responsive control
- Natural steering behavior in both straight and curved segments


#### Summary

- Works well for differential drive robots like TurtleBot3
- Simple yet effective for real-time trajectory tracking
- Easy to tune using `lookahead` distance and gain value (1.5 in this case)

This controller is at the heart of the robot’s ability to follow either raw or smoothed paths while adapting to the robot’s current position in real time.


---

## Project Features

### 📂 Folder Structure
```
bot_ws/
├── src/
│   └── turtlebot3_path_tracking/
│       ├── launch/
│       │   ├── normal_path.launch.py
│       │   └── smoothed_path.launch.py
│       ├── plots/
│       │   └── path_plotter.py
│       ├── test/
│       │   ├── test_controller.py
│       │   ├── test_path_smoother.py
│       │   └── test_trajectory_generator.py
│       ├── turtlebot3_path_tracking/
│       │   ├── main_tracker.py
│       │   └── path_tracker/
│       │       ├── controller.py
│       │       ├── obstacle_avoidance.py
│       │       ├── path_smoother.py
│       │       └── trajectory_generator.py
│       ├── setup.py
│       └── package.xml
```

---

### Launch Instructions

#### 1. Start Gazebo Simulation
Launch an empty Gazebo world with the TurtleBot3 model:

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

---

#### 2. Launch Path Tracker Node

Launch the path tracking controller node with either a normal (raw) path or a smoothed path.

- For **normal (raw)** path tracking:
```bash
ros2 launch turtlebot3_path_tracking normal_path.launch.py
```

- For **smoothed** path tracking:
```bash
ros2 launch turtlebot3_path_tracking smoothed_path.launch.py
```

---

#### 3. RViz Visualization

To visualize the robot, path, and trajectory in RViz:

```bash
rviz2 -d ~/bot_ws/src/turtlebot3_path_tracking/config/path_viz.rviz
```

Make sure this config displays topics like `/odom`, `/cmd_vel`, and optionally `/scan` for obstacle detection.

---

#### 4. Plotting Path Comparison

To generate and view the plot comparing raw waypoints vs. smoothed path:

```bash
cd ~/bot_ws/src/turtlebot3_path_tracking/plots
python3 path_plotter.py
```

This will save and display `path_comparison.png` showing the effectiveness of cubic spline smoothing.


### Testing

Run all tests:
```bash
cd ~/bot_ws/src/turtlebot3_path_tracking
pytest test
```

Includes:
- Path smoothing tests
- Trajectory logic
- Controller response
- Error handling

**All 9 test cases pass!**

---

### Obstacle Avoidance

In addition to trajectory tracking, the system includes a **reactive obstacle avoidance module** defined in `obstacle_avoidance.py`.

It works as follows:

#### Obstacle Detection
- The function `should_avoid_obstacle()` scans the front sector of the LiDAR (`/scan` topic).
- If any object is detected within a **1.0 meter threshold**, the system triggers avoidance behavior.

#### Direction Selection
- The function `choose_turn_direction()` evaluates the total clearance on the left and right sides.
- The robot chooses to turn **toward the side with more open space**.

#### Bypass Waypoint Generation
- The function `get_bypass_waypoint()` computes a new temporary goal:
  - It adds a **forward offset** to keep moving ahead.
  - It adds a **lateral offset** to shift left or right and steer around the obstacle.
- This new point is added to the path so the robot avoids the obstacle **without stopping completely**.

This module is designed for use with the current Pure Pursuit controller.


## Real-World Deployment Strategy

To extend this project to a real TurtleBot3 robot:

- **Localization**: Replace simulated odometry with a real-time localization method such as **AMCL** using a pre-built map.
- **Hardware Interfaces**: Ensure the robot publishes:
  - `/odom` using encoder-based or fused odometry
  - `/scan` using a real LiDAR sensor (e.g., LDS-01)
- **Sensor Calibration**: Calibrate the IMU and LiDAR to reduce noise in pose estimates.
- **Command Rate**: Run the controller at a higher frequency (e.g., 10–20 Hz) for smoother real-world control.
- **Trajectory Tuning**: Tune linear and angular velocity constraints to suit battery voltage and friction.
- **Safety & Fallbacks**: Add emergency stop conditions using bumper or LiDAR thresholds.
- **Battery Awareness**: Limit speed if battery is below threshold using a voltage monitor node.

Optionally, this project can be integrated with the **navigation2 stack** for global planning and goal setting using maps.


---

## AI Tools Used

- **ChatGPT**: Helped with design logic, controller tuning, testing architecture

---


