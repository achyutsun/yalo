---
layout: default
title: "Report 2"
parent: Project
nav_order: 2
---
# Milestone 2: Yalo Mobile Robot 

{: .no_toc }

This page demonstrates the core capabilities of the Just the Docs theme, including navigation, mathematical typesetting, and technical diagrams.

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Kinematics

The TurtleBot is controlled using velocity commands published to the `/cmd_vel` topic:

$$
\mathbf{u} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
=
\begin{bmatrix}
\text{linear.x} \\
\text{angular.z}
\end{bmatrix}
$$

The robot pose is defined as:

$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

### Continuous-Time Motion Model

$$
\begin{aligned}
\dot{x} &= v \cos\theta \\
\dot{y} &= v \sin\theta \\
\dot{\theta} &= \omega
\end{aligned}
$$

### Discrete-Time Motion Model

$$
\begin{aligned}
x_{k+1} &= x_k + v_k \cos\theta_k \Delta t \\
y_{k+1} &= y_k + v_k \sin\theta_k \Delta t \\
\theta_{k+1} &= \theta_k + \omega_k \Delta t
\end{aligned}
$$

---

## 2. Code Implementation

Below is a snippet of the Python code used to process the assignment data.

```python
import numpy as np

def calculate_velocity(displacement, time):
    """Calculates average velocity."""
    return np.divide(displacement, time)

print(f"Result: {calculate_velocity(100, 20)} m/s")

```

---

## 3. Entropy Exploration Algorithm 
The real world is very dynamic and mobile robots inhabit and share the same spaces as humans, encountering moving cars, people, trains, 18-wheelers, strollers, street traffic barricades, construction, daily and numerous human activities. Even inside the home in the living room, humans don't always take the same path to go to the kitchen, moving unpredictably to the left or to the right to go to the same point A to point B in the same room or between rooms. In another scenario let’s take into consideration the static frame, NASA Lunar rover navigating the Lunar Regolith Terrain (LRT). Here in the space exploration Moon rovers, which is completely new, which doesn’t have a map, there is a challenge for mobile robots to move around. Mobile robots need to be provided with a map of the environment it is moving on. In both these scenarios of NASA Lunar rover which has completely new landscape, orientation, craters, rocks, mountains, vast basins all unique as human has not inhabited moon to take its map, and the scenario of living room during superbowl Game or just say the Mobile Robot Lab with crowd of high school students coming to watch the mobile robots will have some sort of unique obstacles of energetic dynamic moving high school students. To navigate our Yalo mobile robot in this unique environment it is quite a challenge. To simplify, the dynamic movement of high school students freezes the frame during the most crowded time of students moving from the hallway to our Mobile robot lab. If we have our Yalo mobile robot to navigate in this hallway, which is a new environment to it and say we haven’t provided the map to navigate Yalo mobile robot in this hallway.

This is where our Entropy Exploration Algorithm comes into play to navigate our Yalo mobile robot by autonomously mapping the new hallway calculating “Frontiers” then selecting the most informative path to reach the destination elevator, moving through the hallway from the Mobile lab.

![Alt text](../assets/images/Architecture_Diagram_Turtlebot_4_Entropy_Exploration_Algorithm.png)
{: width="500" }

*Figure: Architecture diagram of Entropy Exploration algorithm in YALO mobile robot*

% ─────────────────────────────────────────────
### Shannon Binary Entropy per Cell
$$
The binary entropy of a cell with occupancy probability~$p$ is defined as:

\begin{equation}
    H(p) \;=\; -p \log_2 p \;-\; (1 - p)\log_2(1 - p)
\end{equation}

Unknown cells (value $-1$) are mapped to $p = 0.5$, yielding $H = 1.0$~bit (maximum
uncertainty). The entire grid is vectorised with \textsc{NumPy} — no Python loops
over cells.
$$
% ─────────────────────────────────────────────
### Information Gain at Frontier Viewpoint

For a candidate viewpoint~$v$, the information gain is the sum of entropies over all
cells~$c$ within a sensor disc of radius~$R_{\text{sensor}}$:

\begin{equation}
    \mathrm{IG}(v) \;=\; \sum_{c \,\in\, \mathrm{Disc}(v,\, R_{\text{sensor}})} H(c)
\end{equation}

Integration is performed over a circular disc of radius $R = 6\,\mathrm{m}$
(RPLIDAR-A1 range) using an efficient \textsc{NumPy} slice with a circular Boolean
mask — $\mathcal{O}(R^2)$ rather than $\mathcal{O}(W \cdot H)$.

% ─────────────────────────────────────────────
### Distance-Weighted Utility (Elfes-Style)

The utility of a frontier~$f$ balances information gain against travel cost:

\begin{equation}
    U(f) \;=\; \frac{\mathrm{IG}(f)}{1 \;+\; \lambda\, d(\text{robot},\, f)}
\end{equation}

where $d(\text{robot}, f)$ is the Euclidean distance from the robot to frontier~$f$,
and $\lambda = 0.35$ is a tunable parameter exposed via a ROS parameter server.

% ─────────────────────────────────────────────
### Three Key Modules
OccupancyGridManager — wraps nav_msgs/OccupancyGrid with world↔grid coordinate transforms, cell classification (is_free, is_unknown, is_occupied), and the entropy integration disc query.
detect_frontiers() — Wavefront Frontier Detector (WFD): vectorised free/unknown masks → 8-connected shift operations → BFS clustering → size filtering. No Python cell loops in Step 1.
score_frontiers() — calls information_gain_at() per frontier centroid, computes utility, returns list sorted descending by U(f).
$$

### 3.1 Observations

* Observation A: The system remained stable under load.
* Observation B: Latency increased during the second trial.

### 3.2 Conclusion

The experiment met all primary objectives. Future work should focus on optimizing the data pipeline.

---

## 4. Media

You can include images by placing them in the `assets/images/` folder.



---

## 5. Submission Checklist

* [x] Complete Markdown documentation
* [x] Verify LaTeX rendering
* [x] Generate Mermaid flowchart
* [ ] Peer review feedback

# Markdown Features

## Callouts
> This is a note
{: .note }

> This is a warning
{: .warning }

## Buttons
[Main Button](assignment1.html){: .btn .btn-primary }
[Blue Button](assignment2.html){: .btn .btn-blue }
[Blue Button](assignment3.html){: .btn .btn-red }

## Tables

| Header | Header |
| :--- | :--- |
| Cell | Cell |

---
