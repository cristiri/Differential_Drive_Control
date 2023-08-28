# Differential_Drive_Control

This repository contains Matlab code to simulate tracking control and posture stabilization for a differential-drive robot. 

# File description: 

The repository contains two main scripts: 

- Waypoint_Tracking_Controller.m
- Waypoint_Stabilization_Control.m

which implements the algorithms discussed in [1] consisting of tracking control and waypoint stabilization algorithms based on a dynamic feedback linearization technique. The control laws are coded in the functions: "Trajectory_Tracking_law.m"  and "Waypoint_Tracking_law.m" and the algorithms are simulated using the kinematic equations modelling the motion of a differential-drive robot. 


[1] A. De Luca and G. Oriolo, “Modelling and control of nonholonomic mechanical systems,” in Kinematics and Dynamics of Multi-Body Systems. Vienna, Austria: Springer, 1995, pp. 277–342
