#! /usr/bin/env python

from reader import OculusReader
import rospy
import tf
import geometry_msgs.msg    

def main():
    rospy.init_node('oculus_reader')

    oculus_reader = OculusReader()
    twist_pub = rospy.Publisher('/servo_server/cmd_vel', geometry_msgs.msg.TwistStamped, queue_size=1)
    rate = rospy.Rate(500)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
            _, buttons = oculus_reader.get_transformations_and_buttons()
            if 'rightJS' not in buttons or 'rightTrig' not in buttons or 'rightGrip' not in buttons:
                continue

            print(buttons)

            right_js = buttons['rightJS']
            right_trigger = buttons['rightTrig']
            right_grip = buttons['rightGrip']
            
            twist_stamped = geometry_msgs.msg.TwistStamped()
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.twist.linear.x = right_js[1]
            twist_stamped.twist.linear.y = -right_js[0]
            twist_stamped.twist.linear.z = (right_trigger[0] - right_grip[0])
            twist_stamped.twist.angular.x = 0 
            twist_stamped.twist.angular.y = 0 
            twist_stamped.twist.angular.z = 0

            twist_pub.publish(twist_stamped)

    except KeyboardInterrupt:
        print('Ctrl C: Stopping...')
    except Exception as e:
        print('Error:')
        print(e)

    oculus_reader.stop()


if __name__ == '__main__':
    main()