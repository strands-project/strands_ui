#! /usr/bin/env python

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