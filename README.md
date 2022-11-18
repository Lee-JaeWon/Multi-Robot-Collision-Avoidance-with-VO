# Multi-Robot-Collision-Avoidance-with-VO

## Abstract
This repository for multi-robot collision avoidance using the [Velocity Obstacle](https://en.wikipedia.org/wiki/Velocity_obstacle).<br><br>
Localization is performed through [AMCL(Adaptive Monte Carlo localization)](http://wiki.ros.org/amcl) in a saved map made by [Google Cartographer](https://google-cartographer.readthedocs.io/en/latest/).<br><br>
VO can be used to find the speed closest to the desired speed but not to collide. This is implemented using multiple python scripts and launch files.<br><br>

## Video(temp)
[![Video Label](https://img.youtube.com/vi/IEfeJPWc0WE/0.jpg)](https://youtu.be/IEfeJPWc0WE)

## Limitation
This algorithm may require sufficient space. In the narrow space, robots can collide with the wall.<br>
This is because the result of VO(velocity vector) may not follow Path Planning.<br><br>
If dynamic object detection is also performed through LiDAR, human interaction is possible, but this is a different problem.<br><br>
It may be a classical method, and in the ROS1, Local Path Planning is performed in another method.

## Reference
* [MengGuo : RVO for Multi-agent Systems](https://github.com/MengGuo/RVO_Py_MAS)<br>
: This is a simulation for the Holonomic multi-robot system. This repository is implemented with the help of this.
* [Lee-JaeWon : Multi-Turtlebot3-Cartographer-And-Localization](https://github.com/Lee-JaeWon/Multi-Turtlebot3-Cartographer-And-Localization)<br>
: Configuring namespace and TF Tree for multiple robots is difficult.
* [ROBOTIS : turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)<br>
* Fiorini, P., & Shiller, Z. (1998). Motion Planning in Dynamic Environments Using Velocity Obstacles. The International Journal of Robotics Research, 17, 760 - 772.<br>
* Claes, Daniel and Tuyls, Karl (2018) Multi robot collision avoidance in a shared workspace. AUTONOMOUS ROBOTS, 42 (8). 1749 - 1770.
