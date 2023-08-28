#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion

kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.005
kd_angle = 0.005

GOAL_X = 4
GOAL_Y = 5

distance = 0
total_distance = 0
previous_distance = 0
goal_angle = 0
current_angle = 0
rotate_direction = 1
diff_distance = 0
control_signal_distance = 1000.00
previous_angle = 0
total_angle = 0
diff_angle = 0
control_signal_angle = 1000.00
last_angle = 0

rospy.init_node('Move_till_goal')
reached_goal = False
reached_angle = False
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)


def update_pos(msg):
    global reached_goal
    global reached_angle
    global distance
    global goal_angle
    global current_angle
    global rotate_direction
    global total_distance
    global previous_distance
    global diff_distance
    global control_signal_distance
    global diff_angle
    global previous_angle
    global total_angle
    global control_signal_angle
    previous_angle = current_angle
    total_angle += current_angle
    previous_distance = distance
    total_distance += distance
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    diff_angle = goal_angle - previous_angle
    diff_distance = distance - previous_distance
    distance = sqrt(pow(GOAL_X - current_x, 2) + pow(GOAL_Y - current_y, 2))
    control_signal_distance = kp_distance * distance + ki_distance * total_distance + kd_distance * diff_distance
    control_signal_angle = kp_angle * goal_angle + ki_angle * total_angle + kd_distance * diff_angle
    if distance < 0.05:
        reached_goal = True
    # goal_angle = (atan2(GOAL_Y - current_y, GOAL_X - current_x)) if (
    #             (atan2(GOAL_Y - current_y, GOAL_X - current_x)) >= 0) else (
    #             2 * pi - (atan2(GOAL_Y - current_y, GOAL_X - current_x)))

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    current_angle = yaw
    current_angle = current_angle % (2 * pi)
    if current_angle > pi:
        current_angle = current_angle - 2 * pi

    goal_angle = (atan2(GOAL_Y - current_y, GOAL_X - current_x)) - current_angle
    goal_angle = goal_angle % (2 * pi)
    if goal_angle > pi:
        goal_angle = goal_angle - 2 * pi

    if goal_angle >= 0:
        rotate_direction = 1
    else:
        rotate_direction = -1

    if abs(goal_angle) < 0.001:
        reached_angle = True


pose_get = rospy.Subscriber('odom', Odometry, update_pos)

move_cmd = Twist()

while not reached_angle:
    print(f"reached angle? {reached_angle}")
    print(f"goal angle = {goal_angle}")
    print(f"current angle = {current_angle}")
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = (control_signal_angle - current_angle)*rotate_direction
    if move_cmd.angular.z > 0:
        move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
    else:
        move_cmd.angular.z = max(move_cmd.angular.z, -1.5)
    cmd_vel.publish(move_cmd)
    # print("done rotating")
    rate.sleep()
    if reached_angle:
        print(f"reached angle? {reached_angle}")
        print(f"current angle = {current_angle}")
        print(f"goal angle = {goal_angle}")

move_cmd = Twist()

print(f"reached goal? {reached_goal}")
while not reached_goal:
    print(f"reached goal? {reached_goal}")
    print(f"current distance = {distance}")
    print(f"current angle = {current_angle}")
    print(f"goal angle = {goal_angle}")
    move_cmd.angular.z = 0.0
    move_cmd.linear.x = min(control_signal_distance, 0.1)
    cmd_vel.publish(move_cmd)
    rate.sleep()
    if reached_goal:
        print(f"reached goal? {reached_goal}")
        print(f"current distance = {distance}")

cmd_vel.publish(Twist())
