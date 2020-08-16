#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import tf
import json
import math
import actionlib
import actionlib_msgs
import smach
import smach_ros
import roslib.packages
from geometry_msgs.msg    import Twist
from std_msgs.msg         import Float32
from move_base_msgs.msg import MoveBaseGoal


class test_calc_distance():

    def calc_distance(self, my_trans, location_name):
        file_path = roslib.packages.get_pkg_dir('burger_war') + "/location_list/location_list.json"
        file = open(file_path, 'r')
        location_list_dict = json.load(file)
        x = location_list_dict[location_name]["translation"]["x"] - my_trans[0]
        y = location_list_dict[location_name]["translation"]["y"] - my_trans[1]
        distance = math.sqrt(pow(x, 2) + pow(y, 2))
        return distance

    def remake_target_list(self, my_trans, check_points):
        distance_list = []
        for pos in check_points:
            distance_list.append(self.calc_distance(my_trans, pos))
        min_index = distance_list.index(min(distance_list))
        for i in range(min_index):
            check_points.append(check_points[0])
            check_points.pop(0)
        return check_points

if __name__ == "__main__":
    t = test_calc_distance()
    check_points = ["south_center", "south_left", "south_right", "west_right", "west_center", "west_left", "east_left", "east_center", "east_right", "north_center", "north_left", "north_right"]
    my_trans = [-0.01,0.50,0.00]
    print(t.remake_target_list(my_trans, check_points))

