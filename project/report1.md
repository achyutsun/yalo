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
Yalo is a Mobile Robot. Yalo robot when enters the new

$$p(x) = \frac{1}{\sigma\sqrt{2\pi}} e^{-\frac{1}{2}\left(\frac{x-\mu}{\sigma}\right)^2}$$

Where:
- $$\mu$$ is the mean (peak location).
- $$\sigma$$ is the standard deviation (width of the "bell").

---

## 2. Technical Specifications

Below is a snippet of the Python code used to process the assignment data.

```python
import numpy as np

def calculate_velocity(displacement, time):
    """Calculates average velocity."""
    return np.divide(displacement, time)

print(f"Result: {calculate_velocity(100, 20)} m/s")

```

---

## 3. High-Level System Architecture

You can include images by placing them in the `assets/images/` folder.
![Alt text](../assets/images/Yalo_Architecture.png)

*Fig. 1 | Architecture Diagram
Illustration of the information gain exploration architecture for a 2D Turtlebot [a] System architecture integrating sensing, localization, mapping, and planning. [b] Entropy formulation for occupancy grid maps. [c] Frontier detection, where frontier cells are free cells adjacent to unknown regions. [d] Utility optimization maximizing expected information gain through the utility function. U(x) = I(x) -C(x), where (x) is expected mutual information and C(x) is motion cost. *

---

## 4. Submission Checklist

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
