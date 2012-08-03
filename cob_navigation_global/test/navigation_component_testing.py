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

import unittest

class TestNavigation(unittest.TestCase):
    
    def tearDown(self):
        if (self.sub_moveresult!=0):
            self.sub_moveresult.unregister()
    
    def setUp(self):
      
      rospy.sleep(2)
  
      self.goal_reached = False
      
      self.X = (float)(rospy.get_param("goalX"))
      self.Y = (float)(rospy.get_param("goalY"))
      self.Theta = (float)(rospy.get_param("goalTheta"))
      self.mode = rospy.get_param("mode")
      self.tolerance_d = (float)(rospy.get_param("toleranceD"))
      self.tolerance_a = (float)(rospy.get_param("toleranceA"))

      while(rospy.rostime.get_time() == 0.0):
      		#print 'Waiting for initial time publication'
      		time.sleep(0.1)

      
      self.tfL = tf.TransformListener()
      
      rospy.sleep(1)

      if (self.mode== "simple"):
      	self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped)  
      elif(self.mode == "normal"):
      	self.pub_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal)


      self.pub_cancgoal = rospy.Publisher('/move_base/cancel', GoalID)
      self.pub_cmd = rospy.Publisher('/cmd_vel', Twist)
    
      self.sub_moveresult = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.movebase_result_callback)
     
      rospy.sleep(2) 
                   
    def test_navigation(self):   


      	if(self.mode == "simple"):
		self.moveToSimple(self.X,self.Y,self.Theta)
	elif(self.mode == "normal"):
		self.moveTo(self.X, self.Y, self.Theta)
	
	rospy.sleep(20.0)
	
	(trans,rot) = self.tfL.lookupTransform('/map', '/base_link', rospy.Time(0))

    	self.assertTrue(self.goal_reached == True, "The goal position could not be reached%s"%self.goal_reached)

	messageX = (str)(trans[0]) + " " + (str)(self.X) + " "+ (str)(self.tolerance_d)
	messageY = (str)(trans[1]) + " " + (str)(self.Y) + " "+ (str)(self.tolerance_d)
    	
	self.assertTrue(abs(trans[0] - self.X) <= self.tolerance_d, "Error on the X axis position %s"%messageX)
    	self.assertTrue(abs(trans[1] - self.Y) <= self.tolerance_d, "Error on the Y axis position %s"%messageY)



    def movebase_result_callback(self, data):
      
      if (data.status.text == "Goal reached."):
        self.goal_reached = True
        
    def moveToSimple(self, GX,GY,GTh):
      
      goal = PoseStamped()
      goal.header.frame_id = '/map'
      goal.header.stamp = rospy.get_rostime()
      goal.pose.position.x = GX
      goal.pose.position.y = GY
      goal.pose.orientation.z = sin(radians(GTh)/2)
      goal.pose.orientation.w = cos(radians(GTh)/2)
      rospy.sleep(1)
      self.goal_reached = False
      self.pub_goal.publish(goal)
      
      
    def moveTo(self, GX,GY,GTh):
        goal = MoveBaseActionGoal()
        goal.header.frame_id = '/map'
        goal.header.stamp = rospy.get_rostime()
        goal.goal_id.stamp = goal.header.stamp
        goal.goal_id.id = 'goto'
        goal.goal.target_pose.header = goal.header
        goal.goal.target_pose.pose.position.x = GX
        goal.goal.target_pose.pose.position.y = GY
        goal.goal.target_pose.pose.orientation.z = sin(radians(GTh)/2)
        goal.goal.target_pose.pose.orientation.w = cos(radians(GTh)/2)
        
       	rospy.sleep(1)
        self.goal_reached = False
        
        self.pub_goal.publish(goal)   
        
           
if __name__ == '__main__':

    rospy.init_node('test', anonymous=True)
    rostest.rosrun('cob_navigation_global', 'NavigationDiagnostics',
              TestNavigation, sys.argv)
