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

rospy.init_node("burger_war_main_node")#smath_filesでtfを使用するため,init_nodeする前にtf_listerner()があるとエラー
from smach_files          import *

# Global変数
target_location_global = ""


class Commander(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["move", "fight", "commander", "game_finish"])

        self.check_points     = ["south_center", "south_left", "south_right", "west_center", "west_left", "west_right", "north_center", "north_left", "north_right", "east_center", "east_left", "east_right"]
        self.last_notice_time = rospy.Time.now()
        self.is_enemy_close   = False

        self.tf_listener = tf.TransformListener()
        self.sub_enemy   = rospy.Subscriber('robot2enemy', Float32, self.enemy_callback)

        rospy.sleep(1)
        move_base.pub_initialpose_for_burger_war()
        rospy.sleep(1)

    def execute(self, userdata):
        global target_location_global

        try:
            target_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
            source_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
            (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
            length = math.sqrt(pow(trans[0], 2) + pow(trans[1], 2))
            self.is_enemy_close = True if length < 0.90 else False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF lookup error [base_footprint -> enemy_closest]")
            self.is_enemy_close = False

        #各状況に合わせて状態遷移
        #if self.is_enemy_close == True:
        #    return "fight"
        if len(self.check_points) > 0:
            target_location_global = self.check_points[0]
            self.check_points.pop(0)
            return "move"
        else:
            rospy.sleep(0.1)
            return "commander"

    def enemy_callback(self, msg):
        notice_length = 0.90

        if msg.data <= notice_length and self.is_enemy_close == False:
            self.check_points.insert(0, target_location_global)#中断する目標値を再度格納
            move_base.cancel_goal()
            self.last_notice_time = rospy.Time.now() #最後に敵を確認した時刻
            self.is_enemy_close   = True
            self.check_points.reverse() #敵がいた場合は、逆方向の残りのチェックポイントを取りに行く
        elif msg.data > notice_length and self.is_enemy_close == True:
            self.is_enemy_close == False

class Move(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])
        self.pub_twist   = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        global target_location_global

        goal = json_util.generate_movebasegoal_from_locationname(target_location_global)
        overlaytext.publish("Move to " + target_location_global)

        #移動開始
        result = move_base.send_goal_and_wait_result(goal)

        if result == True:
            rospy.loginfo("Turtlebot reached at [" + target_location_global + "].")
            overlaytext.publish("Turtlebot reached at [" + target_location_global + "].")
            #退避動作開始
            send_data = Twist()
            send_data.linear.x = -10
            self.pub_twist.publish(send_data)
            rospy.sleep(2)
        else:
            rospy.loginfo("Moving Failed [" + target_location_global + "].")
            overlaytext.publish("Moving Failed [" + target_location_global + "].")

        return "finish"

#本番では敵の位置をゴールに設定し、取りに行く
class Fight(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])
        self.tf_listener = tf.TransformListener()
        self.pub_twist   = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):

        overlaytext.publish("STATE: Fight")

        try:
            target_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
            source_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
            goal = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF lookup error [base_footprint -> enemy_closest]")
            rospy.sleep(1)
            return "finish"

        #移動開始
        overlaytext.publish("Move to enemy position")
        rospy.loginfo("move to eney position")
        rospy.loginfo(goal)
        result = move_base.send_goal_and_wait_result(goal)
        rospy.sleep(1)

        return "finish"


if __name__ == "__main__":

    rospy.init_node("burger_war_main_node")
    rospy.loginfo("Start burger war main program.")

    sm = smach.StateMachine(outcomes=["Game_finish"])
    with sm:
        smach.StateMachine.add("Commander", Commander(), transitions={"move": "Move", "fight": "Fight", "commander": "Commander", "game_finish": "Game_finish"})
        smach.StateMachine.add("Move",  Move(),  transitions={"finish": "Commander"})
        smach.StateMachine.add("Fight", Fight(), transitions={"finish": "Commander"})

    sis = smach_ros.IntrospectionServer("server", sm, "/BURGER_WAR_TASK")
    sis.start()
    sm.execute()
    sis.stop()
