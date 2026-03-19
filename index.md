---
title: Home
layout: home
nav_order: 1
---

# Yalo Mobile Robot
Team members: Yibo @JerrySyameimaru, Achyut @achyutsun, Long @lhtruong26

Arizona State University RAS-598 Mobile Robotics Class  Spring, 2026

Professor: Vivek Thangavelu, PhD

Yalo is a Mobile Robot. Yalo robot when entering the new environment with the information from LiDAR the Frontier detection, Entropy Exploration Algorithm maps the environment and navigates towards the target destination.

The Yalo mobile robot is designed to autonomously explore and navigate previously unknown environments using LiDAR-based perception and information-theoretic exploration algorithms. By integrating frontier detection, entropy-driven exploration, and autonomous navigation, Yalo incrementally constructs a spatial representation of its surroundings while efficiently moving toward designated target destinations.

The system leverages real-time LiDAR sensing and odometry data to identify unexplored regions of the environment. Using a frontier-based exploration strategy, the robot detects boundaries between known and unknown space within the occupancy grid map. These frontiers serve as candidate exploration goals.

![Alt text](../assets/images/Yalo_Architecture.png)
Figure: YALO Mobile Robot Architecture diagram
