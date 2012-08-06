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

import sys

PKG = 'cob_navigation_global'

import roslib; roslib.load_manifest(PKG)
roslib.load_manifest('cob_script_server')
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

from simple_script_server import *

import unittest
#DONE tray down and arm folded(yaml file) 
class TestNavigation(unittest.TestCase):
    
    def setUp(self):
      
      self.X = (float)(rospy.get_param("goalX"))
      self.Y = (float)(rospy.get_param("goalY"))
      self.Theta = (float)(rospy.get_param("goalTheta"))
      self.goal_reached = False
      mode = rospy.get_param("mode")
      if ("topic" in mode):
	      self.mode = "topic"
	      self.topic_name = mode["topic"][0]["topicName"]
      else:
              self.mode = "action"
		
      self.tolerance_d = (float)(rospy.get_param("toleranceD"))
      self.tolerance_a = (float)(rospy.get_param("toleranceA"))

      self.sss = simple_script_server()

      while(rospy.rostime.get_time() == 0.0):
      		time.sleep(0.1)


      self.sss.move("arm","folded")
      self.sss.move("tray","down")
      rospy.sleep(0.2)
 
      self.tfL = tf.TransformListener()
 
    def movebase_result_callback(self, data):
      
      if (data.status.text == "Goal reached."):
        self.goal_reached = True     

    def test_navigation(self):   
	

	if (self.mode == "topic"):

		nameIncomp = 'move_base'	
		nameComp = nameIncomp + self.topic_name

		self.move_pub = rospy.Publisher(nameComp+ '/goal', PoseStamped)

		posMsg = self.moveToSimple(self.X, self.Y, self.Theta)
       		rospy.sleep(1)      
		self.move_pub.publish(posMsg)
		self.sub_moveresult = rospy.Subscriber(nameIncomp+'/result', MoveBaseActionResult, self.movebase_result_callback)

		rospy.sleep(20)

      	elif(self.mode == "action"):

 		self.move_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

		self.move_client.wait_for_server()
	
		posMsg = self.moveToSimple(self.X, self.Y, self.Theta)

      		self.pub_goal =  MoveBaseGoal(posMsg) #DONE make the topic name available as a parameter

       		self.move_client.send_goal(self.pub_goal) 
				
		self.move_client.wait_for_result(rospy.Duration(20.0))

        	result = self.move_client.get_result()
		
		state = self.move_client.get_state()
	
    		if(state == 3):
			self.goal_reached = True

	start_time = rospy.rostime.get_time()

	self.tfL.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(20.0))

	(trans,rot) = self.tfL.lookupTransform('/map', '/base_link', rospy.Time(0))
	
	angles = euler_from_quaternion(rot)

	messageX = (str)(trans[0]) + " " + (str)(self.X) + " "+ (str)(self.tolerance_d)
	messageY = (str)(trans[1]) + " " + (str)(self.Y) + " "+ (str)(self.tolerance_d)
	
	self.assertTrue(self.goal_reached == True, "The goal position could not be reached%s"%self.goal_reached)    	
	self.assertTrue(abs(trans[0] - self.X) <= self.tolerance_d, "Error on the X axis position %s"%messageX)
    	self.assertTrue(abs(trans[1] - self.Y) <= self.tolerance_d, "Error on the Y axis position %s"%messageY)
		
        #DONE add orientation check, tf for quaternions to euler

	messageA = (str)(rot[2]) + " " + (str)(self.Theta) + " "+ (str)(self.tolerance_a)
    	self.assertTrue(abs(rot[2] - self.Theta) <= self.tolerance_a, "Error on the Angle %s"%messageA)

    def moveToSimple(self, GX,GY,GTh):
      
      goal = PoseStamped()
      goal.header.frame_id = '/map'
      goal.header.stamp = rospy.get_rostime()
      goal.pose.position.x = GX
      goal.pose.position.y = GY
      
      quat = quaternion_from_euler(0, 0, GTh)

      goal.pose.orientation.w = quat [3]
      
      return goal
  
    def moveTo(self, GX,GY,GTh):
	goal = PoseStamped()
        goal.header.frame_id = '/map'
        goal.header.stamp = rospy.get_rostime()
        goal.goal.target_pose.header = goal.header
        goal.goal.target_pose.pose.position.x = GX
        goal.goal.target_pose.pose.position.y = GY

	quat = quaternion_from_euler(0, 0, GTh)
        goal.goal.target_pose.pose.orientation.w = quat [3]
	return goal

if __name__ == '__main__':

    rospy.init_node('test', anonymous=True)
    rostest.rosrun('cob_navigation_global', 'NavigationDiagnostics',
              TestNavigation, sys.argv)
