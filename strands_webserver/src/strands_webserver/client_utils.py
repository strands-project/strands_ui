import sys
import os
import roslib
import rospy

from strands_webserver.srv import *
from std_msgs.msg import String

""" switch the webserver to server pages from dir """
def set_http_root(dir, prefix='/strands_webserver'):
	rospy.wait_for_service(prefix + '/set_http_root')
 	set_http_root = rospy.ServiceProxy(prefix + '/set_http_root', SetRoot)
	try:
  		resp = set_http_root(dir)
	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))
              


""" tell server to display the page relative to server root """
def display_relative_page(displayNo, page, prefix='/strands_webserver'):
	rospy.wait_for_service(prefix + '/display_page')
	rospy.wait_for_service(prefix + '/get_hostname')
 	display_page = rospy.ServiceProxy(prefix + '/display_page', SetDisplay)
 	get_hostname = rospy.ServiceProxy(prefix + '/get_hostname', GetServerAddress)
	try:
		resp = get_hostname()
		server = 'http://%s:%d/' % (resp.hostname, resp.port) 
  		resp = display_page(displayNo, server + page)
	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))

""" tell server to display an arbitrary url -- this must start with http """
def display_url(displayNo, url, prefix='/strands_webserver'):
	rospy.wait_for_service(prefix + '/display_page')
 	display_page = rospy.ServiceProxy(prefix + '/display_page', SetDisplay)
 	print display_page
	try:
  		resp = display_page(displayNo, url)
	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))

""" tell server to display the provided html content """
def display_content(displayNo, html, prefix='/strands_webserver'):
	rospy.wait_for_service(prefix + '/display_page')
 	display_page = rospy.ServiceProxy(prefix + '/display_page', SetDisplay)
	try:
  		resp = display_page(displayNo, html)
	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))

