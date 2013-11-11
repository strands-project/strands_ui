#! /usr/bin/env python

import roslib
import rospy

import strands_webserver.client_utils


if __name__ == '__main__':
	rospy.init_node("test_client")

	# display some content
	displayNo = rospy.get_param("~display", 0)	

	if displayNo == 0:
		rospy.loginfo('writing to all displays)')
	else:
		rospy.loginfo('writing to display: %s', displayNo)

	# switch the server to display relative to 
	strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('marathon_touch_gui'))
	# and ask it show a page
	strands_webserver.client_utils.display_relative_page(displayNo, 'index.html')
