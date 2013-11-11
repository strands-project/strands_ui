#! /usr/bin/env python

import roslib
import rospy

import strands_webserver.page_utils
import strands_webserver.client_utils

if __name__ == '__main__':
	rospy.init_node("demo")

	name = '<p id="tts">Test</p>'
	buttons = [('Button %i' % i, 'service_%i' % i) for i in range(2)]
	service_prefix = '/server'
	content = strands_webserver.page_utils.generate_button_page(name, buttons, service_prefix)
	print content
	strands_webserver.client_utils.display_content(0, content)

