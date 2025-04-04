# Motion Planning with OMPL and Universal Robots in MuJoCo

This repository contains code for motion planning using [OMPL](https://ompl.kavrakilab.org/) (Open Motion Planning Library) with Universal Robots, simulated in [MuJoCo](https://mujoco.org/).

The script also can control the joint of the robot from MuJoCo's GUI

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

### 3. Run with docker

```bash
docker compose up motion_planner 
```

