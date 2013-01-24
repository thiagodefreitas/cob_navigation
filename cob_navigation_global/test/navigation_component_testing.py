#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: cob_object_detection
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: August 2012
#
# \brief
# Implements Tests for the Navigation Component
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################

#TODO: Make the new implementation

import sys

PKG = 'cob_navigation_global'

import roslib; roslib.load_manifest(PKG)
roslib.load_manifest('cob_script_server')
roslib.load_manifest('cob_gazebo_worlds')
import actionlib
import rostest
import rospy
import tf

from math import *

import time

from tf.msg import tfMessage 
from tf.transformations import euler_from_quaternion

from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import *
from nav_msgs.srv import *

from gazebo.srv import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

from simple_script_server import *
import yaml
import unittest
#DONE tray down and arm folded(yaml file) 
class TestNavigation(unittest.TestCase):

    def setUp(self):

        self.goals = rospy.get_param("goals")
        self.PKG = PKG
        self.goals_qty = len(self.goals)
        self.collisions_ctr = 0
        self.goal_reached = False
        self.mode = rospy.get_param("mode")
        self.action_name = rospy.get_param("action_name")
        self.xy_goal_tolerance  = (float)(rospy.get_param("xy_goal_tolerance"))
        self.yaw_goal_tolerance = (float)(rospy.get_param("yaw_goal_tolerance"))
        self.current_rotation = 0.
        self.dir_chg = 0
        
        self.log_file = open("component_log.yaml", "w")
        self.log = {}
        self.log[self.PKG] = {}
        self.log[self.PKG]["simulation"] = {}
        
        self.sss = simple_script_server()
               
                
        while(rospy.rostime.get_time() == 0.0):
            time.sleep(0.1)


        rospy.sleep(0.5)

        self.sss.move("arm","folded")
        self.sss.move("tray","down")

        self.tfL = tf.TransformListener()
        rospy.Subscriber("/base_bumper/state", ContactsState, self.callback, queue_size=1)                
        
        if (self.mode == "topic"):
            self.topic_name = rospy.get_param("topic_name")
            self.move_pub = rospy.Publisher(self.action_name + self.topic_name + '/goal', PoseStamped)
            self.sub_moveresult = rospy.Subscriber(self.action_name + '/result', MoveBaseActionResult, self.movebase_result_callback)

        elif(self.mode == "action"):

            self.move_client = actionlib.SimpleActionClient(self.action_name, MoveBaseAction)

            self.move_client.wait_for_server()

        self.start_time = 0
        self.elapsed_time = 0


    def callback(self, msg):
        if (msg.states != []):
            self.collisions_ctr+=1

    def movebase_result_callback(self, data):

        if (data.status.text == "Goal reached."):
            self.goal_reached = True

    def test_navigation(self):

        self.start_time = rospy.rostime.get_time()

        for i in range(self.goals_qty):

            self.goal_reached == False
            self.goal_x = self.goals[i][0]
            self.goal_y = self.goals[i][1]
            self.goal_theta = self.goals[i][2]

            if (self.mode == "topic"):

                posMsg = self.navigationGoal(self.goal_x, self.goal_y, self.goal_theta)
                rospy.sleep(1)
                self.move_pub.publish(posMsg)

                rospy.sleep(20)

            elif(self.mode == "action"):

                posMsg = self.navigationGoal(self.goal_x, self.goal_y, self.goal_theta)

                self.pub_goal =  MoveBaseGoal(posMsg) #DONE make the topic name available as a parameter

                self.move_client.send_goal(self.pub_goal)

                self.move_client.wait_for_result(rospy.Duration(30.0))

                result = self.move_client.get_result()

                state = self.move_client.get_state()

                if(state == 3):
                    self.goal_reached = True

            start_time = rospy.rostime.get_time()

            self.tfL.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(20.0))

            (trans,rot) = self.tfL.lookupTransform('/map', '/base_link', rospy.Time(0))

            angles = euler_from_quaternion(rot)

            messageX = (str)(trans[0]) + " " + (str)(self.goal_x) + " "+ (str)(self.xy_goal_tolerance)
            messageY = (str)(trans[1]) + " " + (str)(self.goal_y) + " "+ (str)(self.xy_goal_tolerance)

            self.assertTrue(self.goal_reached == True, "The goal position could not be reached%s"%self.goal_reached)
            self.assertTrue(abs(trans[0] - self.goal_x) <= self.xy_goal_tolerance, "Error on the X axis position %s"%messageX)
            self.assertTrue(abs(trans[1] - self.goal_y) <= self.xy_goal_tolerance, "Error on the Y axis position %s"%messageY)

            #DONE add orientation check, tf for quaternions to euler

            messageA = (str)(rot[2]) + " " + (str)(self.goal_theta) + " "+ (str)(self.yaw_goal_tolerance)
            self.assertTrue(abs(rot[2] - self.goal_theta) <= self.yaw_goal_tolerance, "Error on the Angle %s"%messageA)

            rot_chg = abs(self.current_rotation - rot[2])
            self.current_rotation = rot[2]

            if(rot_chg >= 0.02):
                self.dir_chg += 1

            rospy.sleep(1)

        self.elapsed_time = rospy.rostime.get_time() - self.start_time
        
        self.log[self.PKG]["simulation"]["elapsed_time"] = self.elapsed_time
        self.log[self.PKG]["simulation"]["number_collisions"] = self.collisions_ctr
        self.log[self.PKG]["simulation"]["direction_changes"] = self.dir_chg

        yaml.dump(self.log, self.log_file,default_flow_style=False)

        self.log_file.close()


    def navigationGoal(self, GX,GY,GTh):

        goal = PoseStamped()
        goal.header.frame_id = '/map'
        goal.header.stamp = rospy.get_rostime()
        goal.pose.position.x = GX
        goal.pose.position.y = GY

        quat = quaternion_from_euler(0, 0, GTh)

        goal.pose.orientation.w = quat [3]

        return goal


if __name__ == '__main__':

    rospy.init_node('test', anonymous=True)
    rostest.rosrun('cob_navigation_global', 'NavigationDiagnostics',
            TestNavigation, sys.argv)
