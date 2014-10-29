#! /usr/bin/env python

import rospy

from human_help_manager.srv import AskHelp, AskHelpRequest, Register

import actionlib
from mary_tts.msg import maryttsAction, maryttsGoal

    
class NavHelpSpeech(object):

    def __init__(self):

        self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        self.speak_goal= maryttsGoal()
        rospy.loginfo("Waiting for marytts action...") 
        self.speaker.wait_for_server()
        rospy.loginfo("Done") 
        
        self.service_name='/monitored_navigation/human_help/speech'
        
        rospy.loginfo("Waiting for registration services...") 
        rospy.wait_for_service('/monitored_navigation/human_help/register')
        rospy.wait_for_service('/monitored_navigation/human_help/unregister')
        rospy.loginfo("Done")
        
        self.register=rospy.ServiceProxy('/monitored_navigation/human_help/register', Register)
        self.unregister=rospy.ServiceProxy('/monitored_navigation/human_help/unregister', Register)
        
        self.help_service=rospy.Service(self.service_name, AskHelp, self.help_callback)
        
        self.register('speech',self.service_name)
        
        self.previous_interaction=AskHelpRequest.HELP_FINISHED

    
    
    def help_callback(self,req):
        if req.interaction_status==AskHelpRequest.ASKING_HELP:
            if req.failed_component==AskHelpRequest.NAVIGATION:
                self.speak_goal.text='I am having problems moving. Please push me to a clear area.'
            elif req.failed_component==AskHelpRequest.BUMPER:
                 self.speak_goal.text='My bumper is being pressed. Please release it so I can move on!'                                                 
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()       
        elif  req.interaction_status==AskHelpRequest.HELP_FAILED:
            self.speak_goal.text='Something is still wrong. Are you sure I am in a clear area?'
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()
        elif req.interaction_status==AskHelpRequest.HELP_FINISHED and self.previous_interaction==AskHelpRequest.BEING_HELPED:
            self.speak_goal.text='Thank you! I will be on my way.'
            self.speaker.send_goal(self.speak_goal)
            self.speaker.wait_for_result()
        self.previous_interaction=req.interaction_status
        return 'success'
    
    
    def shutdown_callback(self):
        self.unregister('speech',self.service_name)
    
        
    def main(self):
        rospy.on_shutdown(self.shutdown_callback)      
        rospy.loginfo("Node " + rospy.get_name() + " is now spinning.")
        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    
    rospy.init_node('human_help_speech') 
    speech_help =  NavHelpSpeech()
    speech_help.main()
