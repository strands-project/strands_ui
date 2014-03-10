#! /usr/bin/env python

import os

import rospy


from strands_navigation_msgs.srv import AskHelp
from strands_navigation_msgs.srv import AskHelpRequest

from strands_navigation_msgs.srv import Register


import nav_help_screen.client


    
class NavHelpScreen(object):

    def __init__(self):

        server_host = rospy.get_param("~host_ip", "127.0.0.1")
        server_port = rospy.get_param("~port", 8090)
    
        self.displayNo = rospy.get_param("~display", 0)
        os.system('xdg-open http://' + server_host + ':' + str(server_port))
        rospy.sleep(5)
        nav_help_screen.client.init_nav_help_gui()
        nav_help_screen.client.display_main_page(self.displayNo)
        
        self.previous_interaction=AskHelpRequest.HELP_FINISHED
        
        
        self.service_name='/monitored_navigation/human_help/screen'
        
        self.register=rospy.ServiceProxy('/monitored_navigation/human_help/register', Register)
        self.unregister=rospy.ServiceProxy('/monitored_navigation/human_help/unregister', Register)
        self.help_service=rospy.Service(self.service_name, AskHelp, self.help_callback)
        
        self.register('screen',self.service_name)


        
    def help_callback(self,req):
        service_prefix='/monitored_navigation'
        on_completion=req.interaction_service
        if not self.previous_interaction == req.interaction_status:
            if req.interaction_status==AskHelpRequest.ASKING_HELP:
                if req.failed_component==AskHelpRequest.NAVIGATION:
                    nav_help_screen.client.nav_fail(self.displayNo, service_prefix, on_completion)
                elif req.failed_component==AskHelpRequest.BUMPER:
                    nav_help_screen.client.bumper_stuck(self.displayNo, service_prefix, on_completion)
            elif  req.interaction_status==AskHelpRequest.BEING_HELPED:
                nav_help_screen.client.being_helped(self.displayNo, service_prefix, on_completion)               
            elif req.interaction_status==AskHelpRequest.HELP_FINISHED:
                nav_help_screen.client.display_main_page(self.displayNo)
            elif  req.interaction_status==AskHelpRequest.HELP_FAILED:
                if req.failed_component==AskHelpRequest.BUMPER:
                    nav_help_screen.client.bumper_stuck(self.displayNo, service_prefix, on_completion)
                
            self.previous_interaction=req.interaction_status
        return "success!"
    
    
    def shutdown_callback(self):
        self.unregister('screen',self.service_name)
    
        
    def main(self):
        rospy.on_shutdown(self.shutdown_callback)
        
        
        
        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    
    rospy.init_node('human_help_screen') 
    screen_help =  NavHelpScreen()
    screen_help.main()
