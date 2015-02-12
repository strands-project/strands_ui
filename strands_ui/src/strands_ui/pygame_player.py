import time, pygame.mixer as mixer
import rospy

ANNOUNCE_TOPIC='/pygame_player_negotiation'

class PyGamePlayer:

    def __init__(self, min_vol, max_vol, priority,
                 frequency=22050, size=-16, channels=2, id=1):
        mixer.init(frequency, size, channels)
        self.mixer = mixer.Channel(id)
        self.announcer_pub = rospy.publisher(ANNOUNCE_TOPIC)
        self.subscriber_pub = rospy.subscriber(ANNOUNCE_TOPIC, self.announced())

    def announced(self, data):



    def announce_play_start():

    def play_sound(self, s):
        self.announce_play_start()
        try:
            s.play()
        finally:
            self.announce_play_stop()

