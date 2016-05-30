#!/usr/bin/env python
import roslib; roslib.load_manifest("robot_talk")
import rospy

from robot_talk.msg import RobotTalk
from robot_talk.proxy import RobotTalkProxy

if __name__=="__main__":

    rospy.init_node("robot_talk_example")

    print
    
    proxy = RobotTalkProxy('robot_talk_example')
    
    # Setup (not needed if you are using a configured DB)
    id1 = proxy.add("Greeting", "Hi!", 1.0)
    id2 = proxy.add("Greeting", "Hello!", 2.0)
    id3 = proxy.add("Greeting", "Hey!", .5)

    print     

    proxy.list("Greeting")

    print
    
    # Example 1: Get random text for a given topic
    print "EXAMPLE 1: Get random text for a given topic. Here 'Greeting'"
    for i in range(10):
        text = proxy.get_random_text("Greeting")
        print "Text:", text
        print

    # Example 2: Play random text (via marytts) for a given topic
    print "EXAMPLE 2: Playing random text (via marytts) for a given topic"
    proxy.play_random("Greeting")

    # Tidy up
    proxy.remove(id1)
    proxy.remove(id2)
    proxy.remove(id3)


    

    


    
