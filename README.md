# TES-CVIDS: A Transmission Efficient Sub-map based Collaborative Dense VI-SLAM Framework

# 1. Prerequisites
We have tested the library in **Ubuntu 16**, but it should be easy to compile in other platforms.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## ROS-Kinetic
CVIDS is implemented based on ROS-Kinetic. More details can be found in http://wiki.ros.org/kinetic/Installation.

## Sophus
This is an Lie algebra library. More details can be found in https://github.com/strasdat/Sophus.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **We use 3.4.1, but it should also work for other version at least 3.0**.

## Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org.

## ceres-solver
We use [ceres](http://www.ceres-solver.org/) library to perform non-linear optimizations.



