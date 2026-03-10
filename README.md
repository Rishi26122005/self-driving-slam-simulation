# 🚗 Self-Driving Robot Simulation using SLAM

![Python](https://img.shields.io/badge/Python-3.x-blue)
![Robotics](https://img.shields.io/badge/Field-Robotics-green)
![SLAM](https://img.shields.io/badge/Algorithm-SLAM-orange)
![Path Planning](https://img.shields.io/badge/Algorithm-A*%20Path%20Planning-purple)
![Visualization](https://img.shields.io/badge/Visualization-VPython-red)


## Project Overview

This project is a simulation of a self-driving robot designed to demonstrate the core ideas behind **Simultaneous Localization and Mapping (SLAM)**.  
The robot navigates inside a virtual environment filled with obstacles while continuously estimating its own position and building a map of the surroundings.

The system is implemented in **Python** and visualized using **VPython**, allowing real-time rendering of the robot, environment objects, particles, and LiDAR rays.  
The robot can operate in two modes:

- **Manual mode** – controlled using keyboard inputs  
- **Autonomous mode** – automatically navigates toward a goal using path planning

Once the robot reaches the goal position, it automatically plans a return path back to the starting point while continuing to update the SLAM map.

This project was developed as part of the **Introduction to AI Robotics course** at **Amrita Vishwa Vidyapeetham**.

---

## Simulation Demo

A short video of the simulation running in the environment:

![Simulation Demo](assets/output_video.mp4)

---

## Environment and Robot Simulation

The robot operates in a **2D grid-based environment** containing different obstacles such as boxes, cylinders, and spheres.  
These objects form a navigation space where the robot must detect obstacles and plan safe paths.

![Simulation](assets/screenshots/simulation%20-1.png)

The simulation environment includes:

- Road surface of size **900 × 900 units**
- Boundary walls
- Static obstacles placed at different positions
- A goal location that the robot must reach

---

## LiDAR Based Environment Sensing

To understand its surroundings, the robot uses a **simulated LiDAR sensor**.

The LiDAR emits **24 rays** around the robot and measures the distance to nearby obstacles.  
These distance measurements are then used to update the SLAM map and estimate the robot’s position.

![LiDAR Scan](assets/screenshots/LIDAR%20SCAN.png)

This process allows the robot to detect free space and obstacles in real time.

---

## Navigation Process

### Starting Position

The robot begins at the center of the environment and starts scanning its surroundings.

![Start](assets/screenshots/Initial%20to%20Goal(starting).png)

---

### Moving Toward the Goal

Using the **A* path planning algorithm**, the robot calculates the shortest path toward the goal location.

![Goal Navigation](assets/screenshots/Initial%20to%20Goal%20(Starting)-2.png)

The robot continuously updates its orientation and position while avoiding obstacles detected by the LiDAR.

---

### Reaching the Goal

Once the robot reaches the goal location, the system automatically switches the navigation target to the starting point.

![Goal Reached](assets/screenshots/Reached%20GOAL.png)

---

### Returning to Start

After reaching the goal, the robot calculates a return path and navigates back to the initial position.

![Returning](assets/screenshots/Returning%20to%20start.png)

This demonstrates a **complete navigation cycle**.

---

## SLAM Methodology

The system combines several robotics concepts to perform autonomous navigation.

### Particle Filter Localization

The robot uses a **particle filter with 50 particles** to estimate its position and orientation.  
Each particle represents a possible state of the robot, and their weights are updated based on LiDAR observations.

### Occupancy Grid Mapping

The environment is represented as a **100 × 100 grid map**, where each cell indicates:

- Free space
- Occupied space (obstacles)

LiDAR readings are used to update these cells while the robot moves through the environment.

### Path Planning

The **A* algorithm** is used for computing optimal paths between the robot's current position and the goal location.

---

## Results

The simulation demonstrates reliable navigation in a static environment.

Key observations from the experiments:

- Average localization error: **3–5 units**
- Robot successfully reached the goal and returned to start
- Particle filter converged effectively near obstacles
- LiDAR scanning provided accurate obstacle detection

The final SLAM map closely reflects the environment layout with approximately **90% mapping accuracy**. :contentReference[oaicite:1]{index=1}  

---

## Technologies Used

- Python
- VPython (3D visualization)
- NumPy
- Matplotlib
- A* Pathfinding Library

---

## Project Structure
repo
│
├── src
│ ├── main.py
│ ├── slam.py
│ └── envir.py
│
├── assets
│ ├── screenshots
│ │ ├── simulation images
│ │ ├── lidar scan
│ │ └── navigation stages
│ └── ouput_video.mp4
│
├── requirements.txt
└── README.md

---

## Controls

| Key | Action |
|----|------|
| ↑ | Move forward |
| ↓ | Move backward |
| ← | Turn left |
| → | Turn right |
| Space | Toggle manual / autonomous mode |

---

## Limitations

The simulation focuses on clarity and educational value, so some real-world complexities are simplified.

Current limitations include:

- Static environment (no dynamic obstacles)
- Basic motion noise model
- Fixed LiDAR configuration
- Limited grid resolution

---

## Future Improvements

Possible extensions to the project include:

- Dynamic obstacles
- More realistic sensor noise models
- Higher resolution mapping
- Integration with real robotic hardware
- 3D SLAM extensions

---

## Authors

Rushindra Kalyanam  
Ranesh Katta  
Revanth Kumar  
