# LiDAR-Based Autonomous Obstacle Avoidance Robot
### ROS 2 Jazzy | Gazebo Harmonic | Python | LiDAR

## Project Overview

This project implements an autonomous mobile robot capable of navigating an environment while avoiding nearby obstacles using LiDAR sensor data.

Built using ROS 2 Jazzy and Gazebo Harmonic, the robot continuously monitors its surroundings, detects obstacles, evaluates available space, and changes its direction to avoid collisions.

The system uses a reactive obstacle-avoidance approach rather than map-based navigation or predefined destination coordinates.

The primary objective is to demonstrate autonomous robot movement, sensor integration, real-time obstacle detection, and basic decision-making in a simulated environment.

## Key Features

- **Autonomous Movement:** The robot moves independently without continuous keyboard control.
- **LiDAR-Based Detection:** Uses LiDAR sensor readings to identify nearby obstacles.
- **Real-Time Obstacle Avoidance:** Detects obstacles in the robot's forward path.
- **Directional Decision-Making:** Evaluates available space on the left and right sides.
- **Dynamic Turning:** Changes direction towards the side with more available clearance.
- **Collision Avoidance:** Stops or changes direction when obstacles are detected.
- **Gazebo Simulation:** Tests robot behaviour in a simulated environment.

## Technologies Used

| Technology | Purpose |
|---|---|
| ROS 2 Jazzy | Robot communication and control |
| Gazebo Harmonic | Robot simulation |
| Python | Obstacle-avoidance logic |
| LiDAR | Environmental sensing |
| RViz2 | Sensor data visualization |
| Ubuntu 24.04 | Development environment |

## System Architecture

The robot follows a reactive control architecture.

```text
        LiDAR Sensor
             |
             v
      /scan Topic
             |
             v
     Obstacle Detection
             |
             v
      Distance Analysis
             |
             v
   Left vs Right Clearance
             |
             v
    Direction Selection
             |
             v
       Motion Control
             |
             v
       /cmd_vel Topic
             |
             v
     Differential Drive
             |
             v
       Robot Movement
```

## How It Works

1. The robot moves forward through the simulated environment.
2. The LiDAR sensor continuously scans the surrounding area.
3. The obstacle-avoidance controller analyses the sensor readings.
4. When an obstacle is detected within the configured safety distance, the robot stops.
5. The controller evaluates the available clearance on the left and right sides.
6. The robot selects a direction with sufficient available space.
7. It turns towards that direction and continues moving.
8. This process repeats continuously as the robot encounters new obstacles.

## Obstacle Avoidance Logic

The system uses a simple reactive decision-making approach.

The controller considers different regions of the LiDAR field of view:

- Front
- Front-left
- Front-right
- Left
- Right

Based on the available distance measurements, it determines whether to continue forward, turn left, turn right, or stop.

This approach allows the robot to respond dynamically to obstacles without requiring a predefined map.

## Simulation Environment

The robot is tested in Gazebo Harmonic, where different obstacles can be placed in its environment to evaluate its behaviour.

The simulation demonstrates:

- Independent robot movement.
- Obstacle detection using LiDAR.
- Direction selection based on available clearance.
- Turning and bypassing obstacles.
- Continued movement after avoiding obstacles.

### Project Demonstration



> **Demonstration Video:** The complete simulation recording is available in the repository at [`media/obstacle_avoidance_demo.mp4`](media/obstacle_avoidance_demo.mp4).

## Limitations

This project implements reactive obstacle avoidance, not complete autonomous navigation.

Some limitations include:

- The robot does not navigate to specific user-defined coordinates.
- It does not maintain or use a global environment map.
- It does not perform global path planning.
- It may become trapped in U-shaped obstacles or dead-end environments.
- It does not guarantee an optimal route through complex environments.

These limitations are expected for a basic reactive obstacle-avoidance system.

## Future Improvements

Potential future developments include:

- Learning ROS 2 Navigation (Nav2).
- Integrating SLAM for environment mapping.
- Implementing localization using AMCL.
- Adding goal-directed autonomous navigation.
- Improving obstacle-avoidance decision-making.
- Exploring more advanced path-planning algorithms.

These are future learning and development objectives, not features of the current implementation.

## Project Objective

The main objective of this project is to understand and implement the fundamental concepts of mobile robotics, including:

- ROS 2 communication.
- LiDAR sensor integration.
- Real-time sensor processing.
- Autonomous motion control.
- Reactive obstacle avoidance.
- Robot simulation using Gazebo.

The project demonstrates how a mobile robot can perceive nearby obstacles and make basic movement decisions using real-time sensor information.

---

**Project Status:** Completed – Reactive Obstacle Avoidance

**Developed using:** ROS 2 Jazzy and Gazebo Harmonic
