#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3

class CmdVelToOdom:
    def __init__(self):
        rospy.init_node('cmdvel_to_odom')

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vth = 0.0

        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.rate = rospy.Rate(30)  # 30 Hz

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def spin(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()

            delta_x = self.vx * math.cos(self.th) * dt
            delta_y = self.vx * math.sin(self.th) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # tf transform broadcaster
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            odom.pose.pose.position = Point(self.x, self.y, 0.)
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            odom.child_frame_id = "base_link"
            odom.twist.twist = Vector3(self.vx, 0, self.vth)

            self.odom_pub.publish(odom)

            self.last_time = current_time
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = CmdVelToOdom()
        node.spin()
    except rospy.ROSInterruptException:
        pass

