import roslib
import rospy


import strands_webserver.client_utils
import strands_webserver.page_utils


""" Do setup stuff """
def init_marathon_gui():
	# switch the server to display relative to 
	strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('marathon_touch_gui'))


""" Display the main GUI page on the given display """
def display_main_page(display_no):
	strands_webserver.client_utils.display_relative_page(display_no, 'index.html')

""" Display the bumper hit message and call the given service when the human gives an ok"""
def bumper_stuck(display_no, service_prefix, on_completion):
	label = 'OK'
	html = 'Help! My bumper is stuck against something. To help, please follow these instructions:' 
	html += '<hr/>'	
	html += '<ol>'
	html += '<li>Push me away from any obstructions into a clear space</li>'
	html += '<li>Press the '+label+' button below</li>'
	html += '</ol>'
	buttons = [(label, on_completion)]
	content = strands_webserver.page_utils.generate_alert_button_page(html, buttons, service_prefix)	
	strands_webserver.client_utils.display_content(display_no, content) 
	

	""" Display the bumper hit message and call the given service when the human gives an ok"""
def nav_fail(display_no, service_prefix, on_completion):
	label = 'OK'
	html = 'Help! I appear to have trouble moving past an obstruction. Please can you help me by:' 
	html += '<hr/>'	
	html += '<ol>'
	html += '<li>Pushing me away from any obstructions into a clear space</li>'
	html += '<li>Press the '+label+' button below</li>'
	html += '</ol>'
	buttons = [(label, on_completion)]
	content = strands_webserver.page_utils.generate_alert_button_page(html, buttons, service_prefix)	
	strands_webserver.client_utils.display_content(display_no, content) 
	