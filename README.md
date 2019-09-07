# robotics-tools
Robotics design and analysis tools.

## transformation_matrices.py ##
Functions for calculating Basic Transformation Matrices in 3D space.  Can be used to calculate homogeneous transformations (rotation and translation) for serial kinematic chain manipulators.

## RobotKinematics.m ##
MATLAB class of functions for analyzing serial kinematic chain manipulators.  Includes functions for 6-DOF forward kinematics, Denavit-Hartenberg (DH) Matrix generation, cubic polynomial trajectory generation, homogeneous transformation matrix generation, and planar arm kinematics.

## ROS/pose_plotter.py ##
Creates listener node for live-plotting x, y, and yaw of robot using matplotlib.  