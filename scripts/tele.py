#!/usr/bin/env python3
# coding=utf-8

from queue import Queue, Empty, Full
import threading
import rospy
import geometry_msgs.msg
from aubolib import *
import numpy as np
import libpyauboi5
from aubo_ros_control.srv import AuboCommand

def move_robot_cmd(robot, target_pose):
    # Extract position and orientation from the target pose
    position = tuple((target_pose.position.x, target_pose.position.y, target_pose.position.z))
    quaternion = tuple((
        target_pose.orientation.w,
        target_pose.orientation.x,
        target_pose.orientation.y, 
        target_pose.orientation.z
    ))

    # Create a client for the /aubo_driver/command service
    command_client = rospy.ServiceProxy('/aubo_driver/command', AuboCommand)

    # Create a request for the service
    request = AuboCommand()
    request.joint1 = position[0]
    request.joint2 = position[1]
    request.joint3 = position[2]
    request.joint4 = quaternion[0]
    request.joint5 = quaternion[1]
    request.joint6 = quaternion[2]
    request.is_last_point = False  # Set this to True if this is the last point in a trajectory

    # Call the service
    response = command_client(request)

    if not response.success:
        print("Error moving to pose: " + response.message)



QUEUE_SIZE = 20
pose_queue = Queue(maxsize=QUEUE_SIZE)

def pose_callback(pose_stamped):
    try:
        pose_queue.put(pose_stamped, timeout=0.1)
    except Full:
        print("[Queue Event] queue is full")
        pose_queue.get()

def process_poses(robot):
    while not rospy.is_shutdown():
        try:
            pose_stamped = pose_queue.get(timeout=0.1)
            target_pose = pose_stamped.pose
            print("[TARGET_POSE]", target_pose)
            move_robot_api(robot, target_pose)
        except Empty:  # if the queue is empty after 1 second
            continue
        except Exception as e:
            print("[Exception]", e)

def mock_api(robot, target_pose):
    joint_status = robot.get_joint_status()  # Get the status of the joints
    if joint_status is None:
        raise Exception("Joint status is invalid")

    ret = robot.move_to_target_in_quaternion((-0.2525729686, 0.351404154669, 0.337908426005), (0.924742346638, 0.170364932223, 0.263046571868, 0.215949397019))
    if ret != RobotErrorType.RobotError_SUCC:
        print("Error moving to pose: " + str(ret))

def move_robot_api(robot, target_pose):
    print("[Robot Event] try to perform the movement to the target pose")
    joint_status = robot.get_joint_status()  # Get the status of the joints
    print("[Robot Event] joint_states:", joint_status)
    if joint_status is None:
        raise Exception("Joint status is invalid")

    # Extract position and orientation from the target pose
    position = tuple((target_pose.position.x, target_pose.position.y, target_pose.position.z))
    orientation = tuple((
        target_pose.orientation.w,
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
    ))
    robot.check_event()
    if robot.rshd >= 0 and robot.connected:
        joint_radian = libpyauboi5.get_current_waypoint(robot.rshd)
        print("[Robot Event] joint_radian:", joint_radian)
        ik_result = libpyauboi5.inverse_kin(robot.rshd, joint_radian['joint'], position, orientation)
        print("[Robot Event] ik_result:", ik_result)
        if ik_result is None:
            logger.error(f"Failed to compute IK for {joint_radian}.")
            return RobotErrorType.RobotError_ControlError
        print("[Robot Event] Trying to move")
        result = libpyauboi5.move_joint(robot.rshd, ik_result["joint"])
        print("[Robot Event] Movement completed:", result)
        if result != RobotErrorType.RobotError_SUCC:
            logger.error("Failed to move")
        else:
            logger.info("Succeeded to move!")
            return RobotErrorType.RobotError_SUCC


logger_init()
logger.info("{0} VR teleop listening...".format(Auboi5Robot.get_local_time()))

Auboi5Robot.initialize()
robot = Auboi5Robot()
handle = robot.create_context()
logger.info("robot.rshd={0}".format(handle))

rospy.init_node('vr_tele', anonymous=True)
rospy.Subscriber("/servo_server/target_pose", geometry_msgs.msg.PoseStamped, pose_callback)


try:
    ip = '192.168.0.4'
    port = 8899
    result = robot.connect(ip, port)

    if result != RobotErrorType.RobotError_SUCC:
        logger.info("Falied to connect to server {0}:{1}.".format(ip, port))
    else:
        robot.project_startup()
        robot.enable_robot_event()
        robot.init_profile()
        JOINT_MAXVELC_1 = 1.2
        JOINT_MAXVELC_2 = 1.0
        JOINT_MAXACC = 2
        joint_maxvelc = (JOINT_MAXVELC_1, JOINT_MAXVELC_1, JOINT_MAXVELC_1, JOINT_MAXVELC_2, JOINT_MAXVELC_2, JOINT_MAXVELC_2)
        joint_maxacc = (JOINT_MAXACC, JOINT_MAXACC, JOINT_MAXACC, JOINT_MAXACC, JOINT_MAXACC, JOINT_MAXACC)
        robot.set_joint_maxacc(joint_maxacc)
        robot.set_joint_maxvelc(joint_maxvelc)
        robot.set_arrival_ahead_blend(0.05)

        # mock_api(robot, None)

        pose_processing_thread = threading.Thread(target=process_poses, args=(robot, ))
        pose_processing_thread.start()

        # Keep the main thread running until ROS is shutdown
        while not rospy.is_shutdown():
            pass

        # Once ROS is shutdown, join the pose processing thread
        pose_processing_thread.join()

        robot.disconnect()
        print("Disconnected")

except KeyboardInterrupt:
    robot.move_stop()

except RobotError as e:
    logger.error("robot Event:{0}".format(e))

finally:
    if robot.connected:
        robot.disconnect()
    Auboi5Robot.uninitialize()
    print("Completed!")
 