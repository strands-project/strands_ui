#! /usr/bin/env python

import sys
import os
import roslib
import rospy

import strands_webserver.page_utils
import strands_webserver.client_utils
import std_srvs.srv 

if __name__ == '__main__':
	rospy.init_node("strands_webserver_simple_index")

	# The display to publish on, defaulting to all displays
	display_no = rospy.get_param("~display", 0)	
	http_root = rospy.get_param("~http_root", '/tmp/www/')	

	# create http_root of it doesn't exists
	if not os.path.isdir(http_root):
		os.makedirs(http_root)

	# tell the webserver to serve pages from here
	strands_webserver.client_utils.set_http_root(http_root)

	# the page to create
	file_name = 'index.html'

	# get some html to put in the page
	args = rospy.myargv(argv=sys.argv)
	if len(args) > 1:
		left_html = args[1]
	else:	
		left_html = "<h1>Hello world</h1>"

	# render this html into a two-column template page. this  can render left, right or script
	strands_webserver.page_utils.generate_two_column_page(file_name, http_root, left = left_html)

	# display the page
	strands_webserver.client_utils.display_relative_page(display_no, file_name)

