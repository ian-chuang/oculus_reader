#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import threading
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('receiver_node', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

group.set_end_effector_link('teleop_link')

# Global variables
target_pose = None
new_pose_available = False

def move_robot_to_pose():
    global target_pose, new_pose_available
    velocity_scaling_factor = 1.0
    acceleration_scaling_factor = 1.0
    group.set_max_velocity_scaling_factor(velocity_scaling_factor)
    group.set_max_acceleration_scaling_factor(acceleration_scaling_factor)

    while not rospy.is_shutdown():
        if new_pose_available:
            # Lock resources
            group.set_pose_target(target_pose)
            # print(target_pose)
            # print(group.get_current_pose())

            plan = group.plan()
            success = group.execute(plan)
            new_pose_available = False  # Reset the flag

def pose_callback(pose_stamped):
    global target_pose, new_pose_available
    target_pose = pose_stamped.pose
    new_pose_available = True

if __name__ == "__main__":
    rospy.init_node('receiver_node', anonymous=True)
    rospy.Subscriber("/servo_server/target_pose", geometry_msgs.msg.PoseStamped, pose_callback)
    
    # Start the separate thread to handle planning and execution
    execution_thread = threading.Thread(target=move_robot_to_pose)
    execution_thread.start()

    rospy.spin()
