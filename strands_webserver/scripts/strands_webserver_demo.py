#! /usr/bin/env python

import sys
import os
import roslib
import rospy

import strands_webserver.page_utils
import strands_webserver.client_utils
import std_srvs.srv 


def trigger_pleading(req):
      print 'please, please help'


def party_time(req):
      print 'woo, off I go baby'


if __name__ == '__main__':
	rospy.init_node("demo")

	# tell the webserver where it should look for web files to serve
	http_root = os.path.join(roslib.packages.get_pkg_dir("strands_webserver"), "data")
	strands_webserver.client_utils.set_http_root(http_root)

	# start with a basic page pretending things are going normally
	strands_webserver.client_utils.display_relative_page(0, 'example-page.html')

	# sleep for 2 seconds
	rospy.sleep(2.)

	# now ask for help

	name = 'Help me, I am <em>stuck</em>'
	# buttons = [('Button %i' % i, 'service_%i' % i) for i in range(2)]
	buttons = [('No', 'trigger_pleading'), ('Sure', 'party_time')]
	service_prefix = '/caller_services'
	content = strands_webserver.page_utils.generate_alert_button_page(name, buttons, service_prefix)
	# content = strands_webserver.page_utils.generate_button_page(name, buttons, service_prefix)
	strands_webserver.client_utils.display_content(0, content) 
	rospy.Service('/caller_services/trigger_pleading', std_srvs.srv.Empty, trigger_pleading) 
	rospy.Service('/caller_services/party_time', std_srvs.srv.Empty, party_time) 
	rospy.spin()