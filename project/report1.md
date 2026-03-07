---
layout: default
title: "Report 1"
parent: Project
nav_order: 1
---

# Milestone 1: Yalo Mobile Robot

{: .no_toc }


Team members: Yibo @JerrySyameimaru, Achyut @achyutsun, Long @lhtruong26

Arizona State University RAS-598 Mobile Robotics Class

Professor: Vivek Thangavelu, PhD

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1. Mission Statement & Scope: 
Yalo is a Mobile Robot. Yalo robot when entering the new environment with the information from LiDAR the Frontier detection, Entropy Exploration Algorithm maps the environment and navigates towards the target destination. 

The Yalo mobile robot is designed to autonomously explore and navigate previously unknown environments using LiDAR-based perception and information-theoretic exploration algorithms. By integrating frontier detection, entropy-driven exploration, and autonomous navigation, Yalo incrementally constructs a spatial representation of its surroundings while efficiently moving toward designated target destinations.

The system leverages real-time LiDAR sensing and odometry data to identify unexplored regions of the environment. Using a frontier-based exploration strategy, the robot detects boundaries between known and unknown space within the occupancy grid map. These frontiers serve as candidate exploration goals.

---

## 2. Technical Specifications

**Robot Platform:** TurtleBot 4

**Kinematic Model:** Differential Drive

**Perception Stack:** LiDAR, RGB-D, IMU

Below is a snippet of the Python code used to process the assignment data.

```python
utility function. U(x) = I(x) -C(x), where (x) is expected mutual information and C(x) is motion cost. 

```

---

## 3. High-Level System Architecture

You can include images by placing them in the `assets/images/` folder.

Mermaid Diagram

![Alt text](../assets/images/Active_Exploration_Bot_Flow.png)

![Alt text](../assets/images/Yalo_Architecture.png)

*Fig. 1 | Architecture Diagram
Illustration of the information gain exploration architecture for a 2D Turtlebot [a] System architecture integrating sensing, localization, mapping, and planning. [b] Entropy formulation for occupancy grid maps. [c] Frontier detection, where frontier cells are free cells adjacent to unknown regions. [d] Utility optimization maximizing expected information gain through the utility function. U(x) = I(x) -C(x), where (x) is expected mutual information and C(x) is motion cost. *

---


## 4. Safety & Operational Protocol

To ensure safe operation of the TurtleBot during autonomous exploration, a software safety layer is implemented to prevent uncontrolled robot motion and potential hardware damage. This protocol includes a **software Deadman Switch** and a **system-wide Emergency Stop (E-Stop)** mechanism that monitor communication, sensing, and navigation status.

---
### 4.1 Software Deadman Switch

A **Deadman Switch** is implemented to prevent unintended robot movement when control communication is interrupted. The navigation stack continuously publishes velocity commands (`/cmd_vel`) to the robot base. If these commands are not received within a specified timeout window, the system assumes a communication failure.

The safety monitor node tracks the timestamp of the last received velocity command. If the timeout threshold is exceeded, the robot is immediately commanded to stop by publishing zero velocities.

**Timeout logic**

```python
if (current_time - last_cmd_vel_time) > DEADMAN_TIMEOUT:
    publish_zero_velocity()
```

#### Typical Parameters

| Parameter | Value | Description |
|-----------|------|-------------|
| DEADMAN_TIMEOUT | 0.5 s | Maximum allowed time without velocity command |
| SAFE_STOP_VELOCITY | 0.0 m/s | Velocity used to halt the robot |

This mechanism ensures the robot does not continue moving if the navigation controller crashes or communication between nodes is lost.

---
### 4.2 Emergency Stop (E-Stop) Conditions

In addition to the Deadman Switch, a **system-wide Emergency Stop (E-Stop)** mechanism is implemented. The E-Stop overrides all navigation commands and forces the robot into a safe idle state when critical failures or safety risks are detected.

The following conditions trigger an E-Stop.

---

#### 1. Communication Failure

If velocity commands or control heartbeats from the navigation stack are not received within the allowed timeout window.
```python
timeout(cmd_vel) > DEADMAN_TIMEOUT
```
---

#### 2. Sensor Failure

If LiDAR data becomes unavailable or outdated, the robot loses the ability to perceive obstacles in the environment.
```python
timeout(/scan) > SENSOR_TIMEOUT
```
---

#### 3. Localization Failure

If odometry or localization updates stop, the robot can no longer safely navigate within the map.
```python
timeout(/odom) > ODOM_TIMEOUT
```
---

#### 4. Collision Risk

If LiDAR detects an obstacle closer than the predefined safety distance threshold.
```python
min(scan_range) < SAFETY_DISTANCE
```
---

### 4.3 E-Stop Behavior

When an E-Stop condition is triggered, the robot immediately halts all motion and disables autonomous navigation.

Safety actions include:

- Overriding `/cmd_vel` with zero velocity
- Pausing navigation and exploration modules
- Logging the safety event for debugging and system recovery

Example stop command:
```python
/cmd_vel
linear.x = 0
angular.z = 0
```
---


      
## 5. Git Infrastructure
[https://github.com/achyutsun/yalo](https://github.com/achyutsun/yalo)

---
