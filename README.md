# Reachy KDL Kinematics node

This package was obtained at the github of Pollen-Robotics and is being used for academic research.

## Getting started

This package is configured for one arm of robot_reachy

### Kinematics computation service

It exposes services for kinematics (forward and inverse) computations:

* **/r_arm/forward_kinematics** ([GetForwardKinematics.srv](../reachy_msgs/srv/GetForwardKinematics.srv)) - Compute the forward kinematics for the right arm. 7 joints should be provided (r_shoulder_pitch, r_shoulder_roll, r_arm_yaw, r_elbow_pitch, r_forearm_yaw, r_wrist_pitch, r_wrist_roll).
* **/r_arm/inverse_kinematics** ([GetInverseKinematics.srv](../reachy_msgs/srv/GetInverseKinematics.srv)) - Compute the inverse kinematics for the right arm.

## Requirements

Please note that in order to work properly, this node requires the "/robot_description" and "/joint_states" topics to be published. Depending on your URDF, only the corresponding kinematics chain and their associated services/topics will be created.

## Install

sudo apt install python3-pykdl# reachy_kdl_kinematics
# reachy_kdl_kinematics
