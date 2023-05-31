#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def move_robot_to_pose(target_pose):
    waypoints = []
    waypoints.append(target_pose)

    (plan, fraction) = group.compute_cartesian_path(waypoints,   # waypoints to follow
                                                    0.01,        # eef_step: end-effector's step in meters
                                                    0.0)         # jump_threshold: avoiding jumps in joint-space 

    return group.execute(plan, wait=True)

def pose_callback(pose_stamped):
    target_pose = pose_stamped.pose
    move_robot_to_pose(target_pose)

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ik_solver_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    group.set_end_effector_link('teleop_link')
    rospy.Subscriber("/servo_server/target_pose", geometry_msgs.msg.PoseStamped, pose_callback)
    rospy.spin()
