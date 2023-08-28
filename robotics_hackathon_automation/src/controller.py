#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
# you need to name this node "controller"


# def callback(data):
#     rospy.log(f"I heard: {data.data}")


from robotics_hackathon_automation.msg import Coordinates
import rospy
# from robotics_hackathon_automation.msg import Coordinates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion

# rospy.init_node("controller")
# publisher = rospy.Subscriber('planned_path', Coordinates, callback)

kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05


distance = 0
total_distance = 0
previous_distance = 0
goal_angle = 1000
current_angle = 0
rotate_direction = 1
diff_distance = 0
control_signal_distance = 0

rospy.init_node('Move_till_goal')
reached_goal = False
reached_angle = False
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

planned_path = []
def callback(data):
    global planned_path
    planned_path = data.points

path = rospy.Subscriber('planned_path', Coordinates, callback)

past_5_coord = [0,0]
past_5_time = 0

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
    global past_5_coord
    global past_5_time
    previous_distance = distance
    total_distance += distance
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    # if msg.twist.twist.linear.x < 0.01:
        # robot is stuck or robot is not moving
    # print(f"current_x = {current_x}")
    # print(f"current_y = {current_y}")
    diff_distance = distance - previous_distance
    distance = sqrt(pow(GOAL_X - current_x, 2) + pow(GOAL_Y - current_y, 2))
    control_signal_distance = kp_distance * distance + ki_distance * total_distance + kd_distance * diff_distance
    if distance < 0.2:
        reached_goal = True
    elif time.time() - past_5_time > 5:
        if sqrt((current_x - past_5_coord[0])**2 +(current_y - past_5_coord[1])**2) < 0.1:
            print("stuck")
            print("Goal: " + str(GOAL_X) + " " + str(GOAL_Y))
            print(" coordinates: " + str(current_x) + " " + str(current_y))
            # exit()
        past_5_time = time.time()
        past_5_coord = [current_x, current_y]
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

    if abs(goal_angle) < 0.1:
        # print(" coordinates: " + str(current_x) + " " + str(current_y))
        reached_angle = True


import time
time.sleep(7)
pose_get = rospy.Subscriber('odom', Odometry, update_pos)

for i in planned_path:
    
    GOAL_X = i.x + 1.79
    GOAL_Y = i.y + 0.66
    reached_goal = False
    reached_angle = False
    print(f"GOAL_X = {GOAL_X}")
    print(f"GOAL_Y = {GOAL_Y}")


    move_cmd = Twist()

    while not reached_angle:
        # print(f"reached angle? {reached_angle}")
        # print(f"goal angle = {goal_angle}")
        # print(f"current angle = {current_angle}")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.2 * rotate_direction
        cmd_vel.publish(move_cmd)
        # print("done rotating")
        rate.sleep()
        # if reached_angle:
        #     print(f"reached angle? {reached_angle}")
        #     print(f"current angle = {current_angle}")
        #     print(f"goal angle = {goal_angle}")

    move_cmd = Twist()

    print(f"reached goal? {reached_goal}")
    while not reached_goal:
        # print(f"reached goal? {reached_goal}")
        # print(f"current distance = {distance}")
        # print(f"current angle = {current_angle}")
        # print(f"goal angle = {goal_angle}")
        move_cmd.angular.z = 0.0
        move_cmd.linear.x = min(control_signal_distance, 0.2)
        cmd_vel.publish(move_cmd)
        rate.sleep()
        if reached_goal:
            print(f"reached goal? {reached_goal}")
            print(f"current distance = {distance}")

    cmd_vel.publish(Twist())
