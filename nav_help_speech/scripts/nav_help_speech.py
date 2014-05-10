#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy


from strands_navigation_msgs.srv import AskHelp
from strands_navigation_msgs.srv import AskHelpRequest

from strands_navigation_msgs.srv import Register


import actionlib
from ros_mary_tts.msg import maryttsAction
from ros_mary_tts.msg import maryttsGoal

    
class NavHelpSpeech(object):

    def __init__(self):

        
        self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        self.speak_goal= maryttsGoal()       
        self.speaker.wait_for_server()
        
        self.service_name='/monitored_navigation/human_help/speech'
        
        self.register=rospy.ServiceProxy('/monitored_navigation/human_help/register', Register)
        self.unregister=rospy.ServiceProxy('/monitored_navigation/human_help/unregister', Register)
        self.help_service=rospy.Service(self.service_name, AskHelp, self.help_callback)
        
        self.register('speech',self.service_name)
        
        self.previous_interaction=AskHelpRequest.HELP_FINISHED

        
    def help_callback(self,req):
        if req.interaction_status==AskHelpRequest.ASKING_HELP:
            if req.failed_component==AskHelpRequest.NAVIGATION:
                self.speak_goal.text='Ich habe mich ein wenig verirrt. Kannst du mich bitte in eine freie Umgebung schieben?'
            elif req.failed_component==AskHelpRequest.BUMPER:
                 self.speak_goal.text='Ich bin in ein Hindernis gefahren. Kannst du mich bitte wegschieben?'                                                 
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()       
        elif  req.interaction_status==AskHelpRequest.HELP_FAILED:
            self.speak_goal.text='Bin noch immer ein wenig verirrt. Bist du sicher, dass ich in einer freien Zone stehe?'
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()
        elif req.interaction_status==AskHelpRequest.HELP_FINISHED and self.previous_interaction==AskHelpRequest.BEING_HELPED:
            self.speak_goal.text='Dankeschön. Ich mach mich wieder auf den Weg.'
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()
        self.previous_interaction=req.interaction_status
        return 'success'
    
    
    def shutdown_callback(self):
        self.unregister('speech',self.service_name)
    
        
    def main(self):
        rospy.on_shutdown(self.shutdown_callback)                
        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    
    rospy.init_node('human_help_speech') 
    speech_help =  NavHelpSpeech()
    speech_help.main()
