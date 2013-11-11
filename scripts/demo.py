#! /usr/bin/env python

import roslib
import rospy


import strands_webserver.client_utils
import marathon_touch_gui.client

""" Display the main GUI page on the given display """
def display_main_page(displayNo):
	strands_webserver.client_utils.display_relative_page(displayNo, 'index.html')


if __name__ == '__main__':
	rospy.init_node("demo")

	# display some content
	displayNo = rospy.get_param("~display", 0)	

	if displayNo == 0:
		rospy.loginfo('writing to all displays)')
	else:
		rospy.loginfo('writing to display: %s', displayNo)

	# switch the server to display relative to 
	strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('marathon_touch_gui'))

	# Show the main page of the GUI
	display_main_page(displayNo)
	

	# generate page that has buttons and calls the provided service with the button presses
	