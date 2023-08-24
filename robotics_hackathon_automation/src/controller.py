#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
#you need to name this node "controller"


def callback(data):
    rospy.log(f"I heard: {data.data}")

import rospy
from robotics_hackathon_automation.msg import Coordinates

rospy.init_node("controller")
publisher = rospy.Subscriber('planned_path', Coordinates, callback)
