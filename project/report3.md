---
layout: default
title: "Report 3"
parent: Project
nav_order: 3
---

# Milestone 3: Yalo Mobile Robot

{: .no_toc }

This report presents the final system design, evaluation, and analysis of the Yalo Mobile Robot project.

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1. Project Overview


Topics to include:

- Project motivation
- Exploration objectives
- Autonomous navigation goals
- System capabilities
- Final mission summary

---

## 2. Graphical Abstract





---

## 3. System Architecture

Overview of the complete robot software and hardware pipeline.

Topics to include:



---

## 4. Algorithm

Description of the core algorithms used in the project.

### 4.1 Robot Kinematics

Basic robot motion model and control equations.

$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

$$
\mathbf{u} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
$$

---

### 4.2 Frontier Detection

Overview of frontier-based exploration and BFS clustering.

Topics to include:

\[
\textbf{Centroid Calculation for a Frontier Cluster}
\]

Given a frontier cluster containing \(N\) frontier cells:

\[
\{(x_1, y_1), (x_2, y_2), \dots, (x_N, y_N)\}
\]

the centroid position is computed as the arithmetic mean of all frontier cell coordinates.

\vspace{0.5em}

\[
x_c = \frac{1}{N} \sum_{i=1}^{N} x_i
\]

\[
y_c = \frac{1}{N} \sum_{i=1}^{N} y_i
\]

where:

\[
(x_c, y_c)
\]

is the centroid position of the frontier cluster.

\vspace{1em}

\[
\textbf{Expanded Form}
\]

\[
x_c = \frac{x_1 + x_2 + \cdots + x_N}{N}
\]

\[
y_c = \frac{y_1 + y_2 + \cdots + y_N}{N}
\]

\vspace{1em}

\[
\textbf{Robot Exploration Usage}
\]

The centroid represents the geometric center of a frontier cluster and is used as the navigation target for autonomous exploration.

Larger frontier clusters are typically prioritized because they correspond to larger unexplored regions.
\]

Goal Yaw Computation

The robot computes the yaw angle so it faces the selected frontier centroid.

$$
\Delta x = x_g - x_r
$$

$$
\Delta y = y_g - y_r
$$

$$
\theta = \operatorname{atan2}(y_g - y_r,\ x_g - x_r)
$$

Where:

- $x_r, y_r$ = robot position
- $x_g, y_g$ = goal position
- $\theta$ = goal yaw angle

The yaw is converted into quaternion form:

$$
q_z = \sin\left(\frac{\theta}{2}\right)
$$

$$
q_w = \cos\left(\frac{\theta}{2}\right)
$$

Final quaternion:

$$
(q_x,\ q_y,\ q_z,\ q_w)
=
(0,\ 0,\ \sin(\theta/2),\ \cos(\theta/2))
$$

---

### 4.3 Entropy Exploration

Overview of entropy-based frontier scoring.

Topics to include:


Example equation:

---

### 4.4 Decision Making

Description of how the robot selects navigation goals.

Topics to include:

---

## 5. Benchmarking & Results

Evaluation of system performance during final mission execution.

### 5.1 Accuracy

Topics to include:



*Figure: Example accuracy evaluation plot.*

---

### 5.2 Error Analysis

Topics to include:



Example equation:


---

### 5.3 Success Rate

Performance evaluation across multiple trials.

| Trial | Result | Notes |
|------|------|------|
| 1 | TBD | TBD |
| 2 | TBD | TBD |
| 3 | TBD | TBD |
| 4 | TBD | TBD |
| 5 | TBD | TBD |

---

### 5.4 Final Mission Video

Link to the final demonstration video.

---

## 6. Ethical Impact Statement

Discussion of ethical considerations related to the robot system.

Topics to include:


---

## 7. Custom Module Code Links

## 7. Custom Module Code Links

Links to the major custom modules and key commits used in the project.

| Team Member | Role | Key Git Commit/PR | Specific File(s) Authorship |
|-------------|------|-------------------|-----------------------------|
| Long | Frontier Detection | [Commit `a645456`](https://github.com/YOUR-REPO/commit/a645456) | `frontier_detector.py`, `frontier_utils.py`, `frontier.rviz` |
| Yibo | Decision Making | [Commit `cc44e18`](https://github.com/YOUR-REPO/commit/cc44e18) | `decision_maker.py`, `frontier_detector.py` |
| Achyut | Entropy Exploration | [Commit `defd9b7`](https://github.com/YOUR-REPO/commit/defd9b7) | `entropy_explorer.py` |


---

## 8. Experimental Analysis

Discussion of experimental observations and validation.

Topics to include:

- Runtime issues
- Sensor stability
- Mapping performance
- Navigation consistency
- Environmental challenges

---

## 9. Project Management

Project organization and contribution summary.

Topics to include:

- Team responsibilities
- GitHub workflow
- Instructor feedback integration
- Development timeline

---

## 10. References



---

## 11. Submission Checklist

