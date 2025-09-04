#!/usr/bin/env python3

import rospy
from mpc_planner_msgs.msg import ObstacleArray, ObstacleGMM, Gaussian
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class StaticTestObstaclePublisher:
    def __init__(self):
        rospy.init_node("test_obstacle_publisher")
        self.pub = rospy.Publisher("/input/obstacles", ObstacleArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_obstacle)

    def publish_obstacle(self, event):
        obstacle_array = ObstacleArray()
        obstacle_array.header.stamp = rospy.Time.now()
        obstacle_array.header.frame_id = "base_link"

        # 하나의 장애물
        obs = ObstacleGMM()
        obs.id = 0
        obs.pose.position.x = 1.0
        obs.pose.position.y = 0.0
        obs.pose.orientation.w = 1.0  # yaw = 0

        gaussian = Gaussian()
        path = Path()
        path.header.frame_id = "base_link"
        path.header.stamp = rospy.Time.now()

        # 하나의 예측 점만 넣음
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

        gaussian.mean = path
        gaussian.major_semiaxis.append(0.5)
        gaussian.minor_semiaxis.append(0.3)

        obs.gaussians.append(gaussian)
        obs.probabilities.append(1.0)

        obstacle_array.obstacles.append(obs)
        self.pub.publish(obstacle_array)

if __name__ == "__main__":
    try:
        StaticTestObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

