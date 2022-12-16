#! /usr/bin/env python

from reader import OculusReader
from tf.transformations import euler_from_matrix, quaternion_from_matrix, euler_matrix
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
import timeit    
import sys

np.set_printoptions(formatter={'float': lambda x: "{:>8}".format("{0:0.3f}".format(x))})

class TransformManager:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.reference_pose_trans = None
        self.reference_pose_rot = None

        self.twist_pub = rospy.Publisher('/servo_server/cmd_vel', geometry_msgs.msg.TwistStamped, queue_size=1)

    def publish_transform(self, transform, save_ref=False):
        translation = transform[:3, 3]
        rotation = quaternion_from_matrix(transform)

        if save_ref:
            self.reference_pose_trans = translation
            self.reference_pose_rot = rotation

        self.tf_broadcaster.sendTransform(translation, rotation, rospy.Time.now(), 'oculus', 'world')

        if self.reference_pose_trans is not None and self.reference_pose_rot is not None:
            self.tf_broadcaster.sendTransform(self.reference_pose_trans, self.reference_pose_rot, rospy.Time.now(), 'oculus_reference', 'world')


    def publish_twist(self):
        
        twist_stamped = geometry_msgs.msg.TwistStamped()
        try:
            lin_vel, ang_vel = self.tf_listener.lookupTwist('oculus', 'oculus_reference', rospy.Time(0), rospy.Duration(0.01))
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.twist.linear.x = -lin_vel[1] *3
            twist_stamped.twist.linear.y = -lin_vel[0] *3
            twist_stamped.twist.linear.z = -lin_vel[2] *3
            twist_stamped.twist.angular.x = 0 #ang_vel[0] / 3
            twist_stamped.twist.angular.y = 0 #ang_vel[1] / 3
            twist_stamped.twist.angular.z = 0 #ang_vel[2] / 3

            self.twist_pub.publish(twist_stamped)

            # print("lin_vel")
            # print(np.array(lin_vel))
            # print("ang_vel")
            # print(np.array(ang_vel))
            
        except Exception as e:
            pass

    def publish_twist_joy(self, buttons):
        
        twist_stamped = geometry_msgs.msg.TwistStamped()
        try:
            right_js = buttons['rightJS']
            right_trigger = buttons['rightTrig']
            right_grip = buttons['rightGrip']
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.twist.linear.x = right_js[1]
            twist_stamped.twist.linear.y = -right_js[0]
            twist_stamped.twist.linear.z = right_trigger[0] - right_grip[0]
            twist_stamped.twist.angular.x = 0 #ang_vel[0] / 3
            twist_stamped.twist.angular.y = 0 #ang_vel[1] / 3
            twist_stamped.twist.angular.z = 0 #ang_vel[2] / 3

            self.twist_pub.publish(twist_stamped)
            
        except Exception as e:
            pass




def main():
    rospy.init_node('oculus_reader')

    oculus_reader = OculusReader(ip_address='192.168.0.134' , port=5555)
    tf_manager = TransformManager()
    rate = rospy.Rate(60)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
            transformations, buttons = oculus_reader.get_transformations_and_buttons()
            if 'r' not in transformations or 'A' not in buttons:
                continue

            right_controller_pose = transformations['r']
            pressed_A = buttons['A']
            tf_manager.publish_transform(right_controller_pose, save_ref=pressed_A)
            tf_manager.publish_twist()
            # tf_manager.publish_twist_joy(buttons)

            


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
