#!/usr/bin/env python
# -*- coding: utf-8 -*- 

"""
Test client for strands_webserver

"""

import sys
import os
import roslib
import rospy

from strands_webserver.srv import *
from std_msgs.msg import String



if __name__ == '__main__':
	rospy.init_node("test_client")

	# display some content
	displayNo = 2

	# send some content to appear in the display

 	rospy.wait_for_service('/strands_webserver/display_page')
 	display_page = rospy.ServiceProxy('/strands_webserver/display_page', SetDisplay)
	try:
  		resp = display_page(displayNo, 'http://www.cs.bham.ac.uk')
	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))
              



