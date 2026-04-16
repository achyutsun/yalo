---
layout: default
title: "Report 2"
parent: Project
nav_order: 2
---
# Report 2: YY

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

## 3. Section & Sub-sections

The sidebar will automatically highlight the section you are currently viewing.

### 3.1 Observations

* Observation A: The system remained stable under load.
* Observation B: Latency increased during the second trial.

### 3.2 Conclusion

The experiment met all primary objectives. Future work should focus on optimizing the data pipeline.

---

## 4. Media

You can include images by placing them in the `assets/images/` folder.

![Alt text](../assets/images/Architecture_Diagram_Turtlebot_4_Entropy_Exploration_Algorithm.png){: width="500" }

*Figure: Architecture diagram of Entropy Exploration algorithm in YALO mobile robot*

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
