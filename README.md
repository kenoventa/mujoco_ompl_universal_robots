# Motion Planning with OMPL and Universal Robots in MuJoCo

This repository contains code for motion planning using [OMPL](https://ompl.kavrakilab.org/) (Open Motion Planning Library) with Universal Robots, simulated in [MuJoCo](https://mujoco.org/).

## Overview

The code integrates motion planning with the **OMPL** planner and **MuJoCo** simulation, allowing real-time control of the **Universal Robots** (UR) through the **MuJoCo GUI**. 

The `main()` function allows for direct control, where you move the real robot from MuJoCo.

`move_robot_joint()` function allows robot to move to certain position using **OMPL** planner

- **MuJoCo** is used to simulate the robot's movements and environment.
- **RTDE** is used to send real-time commands to the actual robot, enabling joint control.

![Gripper Demo](assets/robot.gif)

The setup uses Docker to simplify dependencies and ensure a consistent environment.

## 🔗 Repository

[https://github.com/kenoventa/mujoco_ompl_universal_robots.git](https://github.com/kenoventa/mujoco_ompl_universal_robots.git)

## 📦 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/kenoventa/mujoco_ompl_universal_robots.git
cd mujoco_ompl_universal_robots
```

### 2. Build the docker image

```bash
docker build -t motion_planner .
```

### 3. Allow Docker Container to display GUI
```bash
xhost +local:docker
```

### 4. Run with docker

```bash
docker compose up motion_planner 
```

