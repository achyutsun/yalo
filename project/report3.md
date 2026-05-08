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

Frontier Cluster Centroid Calculation

Given a frontier cluster containing \(N\) frontier cells:

$$
\{(x_1, y_1), (x_2, y_2), \dots, (x_N, y_N)\}
$$

the centroid position is computed as the arithmetic mean of all frontier cell coordinates.

Centroid Equations

$$
x_c = \frac{1}{N} \sum_{i=1}^{N} x_i
$$

$$
y_c = \frac{1}{N} \sum_{i=1}^{N} y_i
$$

where:

$$
(x_c, y_c)
$$

is the centroid position of the frontier cluster.

Expanded Form

$$
x_c = \frac{x_1 + x_2 + \cdots + x_N}{N}
$$

$$
y_c = \frac{y_1 + y_2 + \cdots + y_N}{N}
$$

Exploration Usage

The centroid represents the geometric center of a frontier cluster and is used as the navigation target for autonomous exploration.

Larger frontier clusters are typically prioritized because they correspond to larger unexplored regions.

Goal Yaw Computation

The robot computes the yaw angle so it faces the selected frontier centroid.

$$
\Delta x = x_g - x_r
$$

$$
\Delta y = y_g - y_r
$$

$$
\theta = \text{atan2}(y_g - y_r,\ x_g - x_r)
$$

Where:

$$
x_r,\ y_r = \text{robot position}
$$

$$
x_g,\ y_g = \text{goal position}
$$

$$
\theta = \text{goal yaw angle}
$$

The yaw is converted into quaternion form:

$$
q_z = \sin\left(\frac{\theta}{2}\right)
$$

$$
q_w = \cos\left(\frac{\theta}{2}\right)
$$

Final quaternion:

$$
(q_x, q_y, q_z, q_w)
=
(0, 0, \sin(\theta/2), \cos(\theta/2))
$$

---

### 4.3 Entropy Exploration

Overview of entropy-based frontier scoring.

Topics to include:


Example equation:

---

### 4.4 Decision Making

The decision stage is responsible for turning the frontier set into a single navigation target. In the current implementation, the decision maker reuses the frontier detection and entropy scoring pipeline from `entropy_explorer.py`, then applies an additional motion-cost model before sending the final goal to Nav2. This keeps the exploration behaviour consistent: the robot prefers frontiers with high information gain, but it also avoids goals that are expensive, unstable, or poorly aligned with the current robot heading.

### 4.4.1 Frontier Ranking

Frontiers are first detected from the occupancy grid using the frontier detector. Each frontier is scored using the entropy-based utility produced by `score_frontiers()` in `entropy_explorer.py`:

$$U(f) = \frac{IG(f)}{1 + \lambda \cdot d(robot, f)}$$

where:
- $$IG(f)$$ is the information gain around the frontier centroid,
- $$d(robot, f)$$ is the distance from the robot to the frontier,
- $$\lambda$$ is the distance decay factor.

This ranking favours frontiers that reveal more unknown space, which means the robot is naturally pushed toward regions with higher entropy and lower occupancy certainty.

### 4.4.2 Navigation Feasibility

After entropy ranking, the decision maker evaluates whether a frontier is practical to reach. The final score combines entropy with a motion-cost estimate that reflects the effort needed to move toward the goal. The motion cost includes:

- **Base drain**: a constant energy cost for keeping the robot active.
- **Linear cost**: the travel distance to the centroid.
- **Angular cost**: the heading correction needed to face the target.
- **Startup tax**: an extra penalty when the robot must accelerate from near standstill.

In simplified form:

$$C(f) = C_{base} + w_v \cdot d(robot,f) + w_\omega \cdot |\Delta\theta| + C_{startup}$$

The motion cost is then normalised and subtracted from the entropy reward, so the final target is not only informative but also feasible to navigate efficiently.

### 4.4.3 Safety Constraints

The decision maker also applies safety and stability constraints before publishing a goal. These constraints prevent the robot from oscillating between targets or selecting goals that are awkward to approach:

- **Rear-goal rejection**: centroid goals behind the robot are filtered out using the heading alignment gate.
- **Hysteresis**: if a new goal does not improve the score enough, the previous goal is kept.
- **Same-goal tolerance**: very small goal changes are ignored to avoid unnecessary replanning.
- **Forward-biased motion**: the robot prefers goals in front of its current heading rather than constantly reversing direction.

These constraints make the exploration smoother and reduce unstable switching between nearby frontiers.

### 4.4.4 Goal Updates

The final navigation target is published only after the best frontier survives ranking, motion-cost evaluation, and hysteresis. The selected centroid is then sent to Nav2 through `/goal_pose`. If the robot is already navigating and the new target is too similar to the previous one, the update is skipped to avoid redundant replanning.

The goal update process therefore follows this sequence:

1. Detect frontier clusters from the map.
2. Rank them by entropy utility.
3. Apply motion-cost and safety checks.
4. Choose the highest-scoring forward-facing target.
5. Publish the goal to Nav2.

This approach keeps the exploration policy information-driven while still respecting motion effort, consistency, and runtime safety.

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

https://youtu.be/D1sxXO-z0Kw

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

