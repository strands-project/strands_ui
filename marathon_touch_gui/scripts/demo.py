#! /usr/bin/env python

import roslib
import rospy


import marathon_touch_gui.client



if __name__ == '__main__':
	rospy.init_node("demo")

	# display some content
	displayNo = rospy.get_param("~display", 0)	

	if displayNo == 0:
		rospy.loginfo('writing to all displays)')
	else:
		rospy.loginfo('writing to display: %s', displayNo)

	# Setup -- must be done before other marathon_touch_gui calls
	marathon_touch_gui.client.init_marathon_gui()

	# Show the main page of the GUI
	marathon_touch_gui.client.display_main_page(displayNo)

	rospy.sleep(2)
	
	# All callback services should be under this prefix
	service_prefix = '/patroller'

	# Do something like this on bumper collision

	# when the human gives the 'ok' then /patroller/bumper_recovered is called
	# note that this service must be provided by some other node
	on_completion = 'bumper_recovered'
	marathon_touch_gui.client.bumper_stuck(displayNo, service_prefix, on_completion)

	rospy.sleep(2)

	# Do something like this on move_base failure

	# when the human gives the 'ok' then /patroller/robot_moved is called
	# note that this service must be provided by some other node
	on_completion = 'robot_moved'
	marathon_touch_gui.client.nav_fail(displayNo, service_prefix, on_completion)

	# Return to main page
	marathon_touch_gui.client.display_main_page(displayNo)
	
