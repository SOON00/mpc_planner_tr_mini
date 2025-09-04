#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import yaml
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
from collections import deque
import copy

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0, 0, yaw)

def point_line_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2), (x1, y1)

    t = float((px - x1) * dx + (py - y1) * dy) / float(dx*dx + dy*dy)
    t_clamped = max(0, min(1, t))

    closest_x = x1 + t_clamped * dx
    closest_y = y1 + t_clamped * dy
    dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
    return dist, (closest_x, closest_y)

def interpolate_segment(x1, y1, yaw1, x2, y2, yaw2, resolution=0.05):
    dist = math.hypot(x2 - x1, y2 - y1)
    steps = max(int(dist / resolution), 1)
    path = []

    for i in range(steps + 1):
        t = float(i) / float(steps)
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        yaw = yaw1 + t * (yaw2 - yaw1)

        path.append({'position': [x, y, yaw]})
    return path

def load_topology(yaml_path):
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)

def bfs_path(graph, start, goal):
    queue = deque([[start]])
    visited = set()

    while queue:
        path = queue.popleft()
        node = path[-1]
        if node == goal:
            return path
        if node not in visited:
            visited.add(node)
            for neighbor in graph[node]['edges']:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    return None

def publish_path_from_list(node_list, topic="/topology_path", frame_id="map"):
    pub = rospy.Publisher(topic, Path, queue_size=1, latch=True)
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = rospy.Time.now()

    for node in node_list:
        pos = node['position']
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = 0.0

        q = quaternion_from_yaw(pos[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        path_msg.poses.append(pose)

    pub.publish(path_msg)

def find_nearest_edge_point(graph, x, y):
    min_dist = float('inf')
    nearest_edge = None
    nearest_point = None

    for node_name, node_data in graph.items():
        x1, y1, _ = node_data['position']
        for neighbor in node_data['edges']:
            neighbor_data = graph[neighbor]
            x2, y2, _ = neighbor_data['position']

            dist, proj = point_line_segment_distance(x, y, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                nearest_edge = (node_name, neighbor)
                nearest_point = proj

    return nearest_edge, nearest_point

def generate_and_publish_path_once():
    global graph, tf_listener, goal_position
    try:
        now = rospy.Time(0)
        tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform("map", "base_link", now)
        robot_x, robot_y = trans[0], trans[1]
        robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
    except Exception as e:
        rospy.logerr("TF 오류: {}".format(e))
        return

    graph_mod = copy.deepcopy(graph)

    nearest_edge_start, proj_point_start = find_nearest_edge_point(graph, robot_x, robot_y)
    if nearest_edge_start is None:
        rospy.logwarn("가까운 엣지를 찾지 못했습니다.")
        return
    node_a_start, node_b_start = nearest_edge_start
    virtual_start = "virtual_start"
    graph_mod[virtual_start] = {
        'position': [proj_point_start[0], proj_point_start[1], robot_yaw],
        'edges': [node_a_start, node_b_start]
    }
    graph_mod[node_a_start]['edges'].append(virtual_start)
    graph_mod[node_b_start]['edges'].append(virtual_start)

    goal_x = goal_position.position.x
    goal_y = goal_position.position.y
    goal_q = goal_position.orientation
    goal_yaw = tf.transformations.euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])[2]

    nearest_edge_goal, proj_point_goal = find_nearest_edge_point(graph, goal_x, goal_y)
    if nearest_edge_goal is None:
        rospy.logwarn("목표 위치에 가까운 엣지를 찾지 못했습니다.")
        return
    node_a_goal, node_b_goal = nearest_edge_goal
    virtual_goal = "virtual_goal"
    graph_mod[virtual_goal] = {
        'position': [proj_point_goal[0], proj_point_goal[1], goal_yaw],
        'edges': [node_a_goal, node_b_goal]
    }
    graph_mod[node_a_goal]['edges'].append(virtual_goal)
    graph_mod[node_b_goal]['edges'].append(virtual_goal)

    path_nodes = bfs_path(graph_mod, virtual_start, virtual_goal)
    if not path_nodes:
        rospy.logwarn("BFS 경로를 찾을 수 없습니다.")
        return

    full_path_nodes = []

    interpolated_start = interpolate_segment(
        robot_x, robot_y, robot_yaw,
        proj_point_start[0], proj_point_start[1], robot_yaw,
        resolution=5.0
    )
    full_path_nodes.extend(interpolated_start)

    for i in range(1, len(path_nodes)):
        prev_node = path_nodes[i - 1]
        curr_node = path_nodes[i]

        prev_pos = graph_mod[prev_node]['position']
        curr_pos = graph_mod[curr_node]['position']

        interpolated = interpolate_segment(
            prev_pos[0], prev_pos[1], prev_pos[2],
            curr_pos[0], curr_pos[1], curr_pos[2],
            resolution=5.0
        )
        full_path_nodes.extend(interpolated[1:])

    interpolated_goal = interpolate_segment(
        proj_point_goal[0], proj_point_goal[1], goal_yaw,
        goal_x, goal_y, goal_yaw,
        resolution=5.0
    )
    full_path_nodes.extend(interpolated_goal[1:])

    publish_path_from_list(full_path_nodes, topic="/topology_path")

goal_position = None
previous_goal = None
tf_listener = None
graph = None

def goals_are_same(goal1, goal2, pos_threshold=0.05, yaw_threshold=0.1):
    if goal1 is None or goal2 is None:
        return False
    dx = goal1.position.x - goal2.position.x
    dy = goal1.position.y - goal2.position.y
    dist = math.sqrt(dx*dx + dy*dy)

    yaw1 = tf.transformations.euler_from_quaternion([
        goal1.orientation.x,
        goal1.orientation.y,
        goal1.orientation.z,
        goal1.orientation.w
    ])[2]
    yaw2 = tf.transformations.euler_from_quaternion([
        goal2.orientation.x,
        goal2.orientation.y,
        goal2.orientation.z,
        goal2.orientation.w
    ])[2]
    yaw_diff = abs(yaw1 - yaw2)

    return dist < pos_threshold and yaw_diff < yaw_threshold

def goal_callback(msg):
    global goal_position, previous_goal

    if previous_goal is not None and goals_are_same(msg.pose, previous_goal):
        rospy.loginfo("Same Goal!")
        return

    goal_position = msg.pose
    previous_goal = copy.deepcopy(goal_position)

    rospy.loginfo("Goal: ({:.2f}, {:.2f})".format(goal_position.position.x, goal_position.position.y))
    generate_and_publish_path_once()

def main():
    global graph, tf_listener

    rospy.init_node('topology_global_planner_node')

    graph_yaml_path = rospy.get_param('~topology_yaml')

    graph = load_topology(graph_yaml_path)

    tf_listener = tf.TransformListener()

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

