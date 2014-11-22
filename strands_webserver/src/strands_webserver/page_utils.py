import sys
import os
import roslib
import rospy
import web

template_dir = roslib.packages.get_pkg_dir('strands_webserver') + '/templates'
render = web.template.render(template_dir)


def generate_two_column_page(file_name, target_dir, left = "", right = "", script = ""):
	""" Create a page by rendering the strands_webserver/templates/index.html using web.py and place it at target_dir/file_name """

	full_path = target_dir + file_name

	try:
		os.makedirs(os.path.dirname(full_path))
	except OSError, e:
		pass

	page = str(render.two_col(left, right, script))
	with open(full_path, 'w+') as f:
		f.write(page)

	print 'written ', full_path


"""
Uses web.py to generate html for the body of a page which contains an arbitrary html notice with buttons below. Buttons should be specified as a list of (label, service) tuples where clicking on the button with label will call /service_prefix/service. Html will work in the body of the main page served by strands_webserver.
"""
def generate_button_page(notice, buttons, service_prefix):
	return str(render.buttons_multi_service(notice, buttons, service_prefix))

"""
Uses web.py to generate html for the body of a page which contains a large text notice with buttons below. Buttons should be specified as a list of (label, service) tuples where clicking on the button with label will call /service_prefix/service. Html will work in the body of the main page served by strands_webserver.
"""
def generate_alert_button_page(notice, buttons, service_prefix):
	return generate_button_page('<div class="notice">' + notice + '</div>', buttons, service_prefix)
