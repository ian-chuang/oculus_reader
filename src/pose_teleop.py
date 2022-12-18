#! /usr/bin/env python

from reader import OculusReader
from tf.transformations import euler_from_matrix, quaternion_from_matrix, euler_matrix, translation_matrix, quaternion_matrix, translation_from_matrix, quaternion_from_euler
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
import timeit    
import sys

np.set_printoptions(formatter={'float': lambda x: "{:>8}".format("{0:0.3f}".format(x))})

class PoseManager:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.oculus_ref_trans = None
        self.oculus_ref_rot = None
        self.eef_ref_trans = None
        self.eef_ref_rot = None

        self.pose_pub = rospy.Publisher('/servo_server/target_pose', geometry_msgs.msg.PoseStamped, queue_size=1)

    def publish_transform(self, transform, save_ref=False):
        trans = translation_from_matrix(transform)
        rot = quaternion_from_matrix(transform)

        if save_ref:
            self.oculus_ref_trans = trans
            self.oculus_ref_rot = rot
            self.eef_ref_trans, self.eef_ref_rot = self.tf_listener.lookupTransform(
                'world',
                'teleop_link',
                rospy.Time(0)
            )


        self.tf_broadcaster.sendTransform(
            trans, 
            rot, 
            rospy.Time.now(), 
            'oculus', 
            'world'
        )

        if self.oculus_ref_trans is not None and self.oculus_ref_rot is not None:
            self.tf_broadcaster.sendTransform(
                self.oculus_ref_trans, 
                self.oculus_ref_rot, 
                rospy.Time.now(), 
                'oculus_reference', 
                'world'
            )

        if self.eef_ref_trans is not None and self.eef_ref_rot is not None:
            self.tf_broadcaster.sendTransform(
                self.eef_ref_trans, 
                self.eef_ref_rot, 
                rospy.Time.now(), 
                'eef_reference', 
                'world'
            )

        try:
            tf_trans, tf_rot = self.tf_listener.lookupTransform(
                'oculus_reference',
                'oculus',
                rospy.Time(0)
            )

            self.tf_broadcaster.sendTransform(
                tf_trans, 
                tf_rot, 
                rospy.Time.now(), 
                'target_pose', 
                'eef_reference'
            )

        except Exception as e:
            pass

        

    def publish_pose(self):
        try:
            pose_trans, pose_rot = self.tf_listener.lookupTransform(
                'world',
                'target_pose',
                rospy.Time(0)
            )
        except Exception as e:
            return

        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = 'world'
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = pose_trans[0]
        pose_stamped.pose.position.y = pose_trans[1]
        pose_stamped.pose.position.z = pose_trans[2]
        pose_stamped.pose.orientation.x = pose_rot[0]
        pose_stamped.pose.orientation.y = pose_rot[1] 
        pose_stamped.pose.orientation.z = pose_rot[2]
        pose_stamped.pose.orientation.w = pose_rot[3]

        self.pose_pub.publish(pose_stamped)


class GripperControl():
    def __init__(self):
        self.position_pub = rospy.Publisher('/gripper_position_controller/command', std_msgs.msg.Float64, queue_size=1)

    def publish_position(self, position):
        msg = std_msgs.msg.Float64()
        msg.data = position
        self.position_pub.publish(msg)

    

def main():
    rospy.init_node('oculus_reader')

    oculus_reader = OculusReader(ip_address='192.168.0.134' , port=5555)
    tf_manager = PoseManager()
    gripper_control = GripperControl()
    rate = rospy.Rate(125)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
            transformations, buttons = oculus_reader.get_transformations_and_buttons()
            if 'r' not in transformations or 'A' not in buttons or 'rightTrig' not in buttons:
                continue

            right_controller_pose = transformations['r']
            pressed_A = buttons['A']
            rightTrig = buttons['rightTrig']

            tf_manager.publish_transform(right_controller_pose, save_ref=pressed_A)
            tf_manager.publish_pose()
            gripper_control.publish_position(rightTrig[0] * 0.65)

    except KeyboardInterrupt:
        print('Ctrl C: Stopping...')
    except Exception as e:
        print('Error:')
        print(e)

    oculus_reader.stop()


if __name__ == '__main__':
    main()







# class PoseToTwist:
#     def __init__(self):
#         self.last_lin_vec = None
#         self.last_ang_vec = None
#         self.last_time = None

#     def get_twist(self, lin_vec, ang_vec):
#         if not self.last_lin_vec or not self.last_ang_vec or not self.last_time:
#             self.last_lin_vec = lin_vec
#             self.last_ang_vec = ang_vec
#             self.last_time = timeit.default_timer()
#         else :
#             delta_t = timeit.default_timer() - self.last_time
#             lin_vel = geometry_msgs.msg.Vector3(
#                 (lin_vec.position.x - self.last_lin_vec.position.x)/delta_t,
#                 (lin_vec.position.y - self.last_lin_vec.position.y)/delta_t,
#                 (lin_vec.position.z - self.last_lin_vec.position.z)/delta_t,
#             )
#             ang_vel = geometry_msgs.msg.Vector3(
#                 (ang_vec.position.x - self.last_ang_vec.position.x)/delta_t,
#                 (ang_vec.position.y - self.last_ang_vec.position.y)/delta_t,
#                 (ang_vec.position.z - self.last_ang_vec.position.z)/delta_t,
#             )


#             self.last_time = timeit.default_timer()
