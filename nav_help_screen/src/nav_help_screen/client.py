import roslib
import rospy


import strands_webserver.client_utils
import strands_webserver.page_utils


""" Do setup stuff """
def init_nav_help_gui():
	# switch the server to display relative to 
	strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('nav_help_screen'))


""" Display the main GUI page on the given display """
def display_main_page(display_no):
	strands_webserver.client_utils.display_relative_page(display_no, 'index.html')

""" Display the bumper hit message and call the given service when the human gives an ok"""
def bumper_stuck(display_no, service_prefix, on_completion):
	label = 'Hilf mir'
	#html = 'Help! My bumper is stuck against something. If you can help me, please press the button below.'
        html = 'Hilfe! Ich bin in ein Hindernis gefahren. Wenn du mir helfen magst, bitte dr&uuml;ck den unteren Knopf.'
	#html += '<hr/>'	
	#html += '<ol>'
	#html += '<li>Push me away from any obstructions into a clear space</li>'
	#html += '<li>Press the '+label+' button below</li>'
	#html += '</ol>'
	buttons = [(label, on_completion)]
	content = strands_webserver.page_utils.generate_alert_button_page(html, buttons, service_prefix)	
	strands_webserver.client_utils.display_content(display_no, content)


	

	""" Display the bumper hit message and call the given service when the human gives an ok"""
def nav_fail(display_no, service_prefix, on_completion):
	label = 'Hilf mir'
	#html = 'Help! I appear to have trouble moving past an obstruction. If you can help me, please press the button below.' 
	html = 'Hilfe! Ich habe Probleme mich an den Hindernissen vorbeizubewegen. Wenn du mir helfen magst, bitte dr&uumlck den unteren Knopf.'
	#html += '<hr/>'	
	#html += '<ol>'
	#html += '<li>Pushing me away from any obstructions into a clear space</li>'
	#html += '<li>Press the '+label+' button below</li>'
	#html += '</ol>'
	buttons = [(label, on_completion)]
	content = strands_webserver.page_utils.generate_alert_button_page(html, buttons, service_prefix)	
	strands_webserver.client_utils.display_content(display_no, content) 

	
""" Screen for when a human is helping the robot"""
def being_helped(display_no, service_prefix, on_completion):
        label = 'OK'
        #html = 'Thank you! Please follow these instructions:' 
        html = 'Dankesch&ouml;n. Bitte mache Folgendes:'
	html += '<hr/>' 
        html += '<ol>'
        #html += '<li>Push me away from any obstructions into a clear space</li>'
        #html += '<li>Press the '+label+' button below</li>'
        html += '<li>Schieb mich weg von den Hindernissen in die Mitte des Ganges.</li>'
	html += '<li>Dr&uecke den '+label+' Knopf unten</li>'	
	html += '</ol>'
        buttons = [(label, on_completion)]
        content = strands_webserver.page_utils.generate_alert_button_page(html, buttons, service_prefix)        
        strands_webserver.client_utils.display_content(display_no, content)     
        
        
        
