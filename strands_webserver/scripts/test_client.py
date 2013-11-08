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

""" switch the webserver to server pages from dir """
def set_http_root(dir):
	rospy.wait_for_service('/strands_webserver/set_http_root')
 	set_http_root = rospy.ServiceProxy('/strands_webserver/set_http_root', SetRoot)
	try:
  		resp = set_http_root(dir)
	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))
              


""" tell server to display the page relative to server root """
def display_relative_page(displayNo, page):
	rospy.wait_for_service('/strands_webserver/display_page')
 	display_page = rospy.ServiceProxy('/strands_webserver/display_page', SetDisplay)
	try:
  		resp = display_page(displayNo, 'http://localhost:8090/' + page)
	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))


if __name__ == '__main__':
	rospy.init_node("test_client")

	# display some content
	displayNo = rospy.get_param("~display", 1)	

	rospy.loginfo('writing to display: %s', displayNo)

	# switch the server to display relative to 
	set_http_root(roslib.packages.get_pkg_dir('marathon_touch_gui'))
	# and ask it show a page
	display_relative_page(displayNo, 'index.html')

 	
              



