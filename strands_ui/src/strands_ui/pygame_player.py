import time
import pygame.mixer as mixer
import rospy
from StringIO import StringIO
from strands_ui.msg import SoundPriorities

ANNOUNCE_TOPIC='/pygame_player_negotiation'

class PyGamePlayer:

    def __init__(self, min_vol, max_vol, priority,
                 frequency=22050, size=-16, channels=2, id=1):
        mixer.init(frequency, size, channels)
        self.priority_map = {}
        self.min_vol = min_vol
        self.max_vol = max_vol
        self.priority = priority
        self.channel = mixer.Channel(id)
        self.announcer_pub = rospy.publisher(ANNOUNCE_TOPIC, SoundPriorities)
        self.subscriber_pub = rospy.subscriber(ANNOUNCE_TOPIC, SoundPriorities,
                                               self.announced())

    def announced(self, data):
        self.priority_map[data.id] = data.priority

        max_key, max_value = max(self.priority_map.iteritems(),
                                 key=lambda x: x[1])

        if max_key == rospy.get_name():
            self.channel.set_volume(self.max_vol)
        else:
            if max_value > self.priority:
                self.channel.set_volume(self.min_vol)
            else:
                self.channel.set_volume(self.max_vol)

    def announce_priority(self, prio):
        s = SoundPriorities()
        s.id = rospy.get_name()
        s.priority = prio
        self.announcer_pub.publish(s)

    def play_sound(self, s):
        self.announce_priority(self.priority)
        try:
            self.channel.play(s)
        finally:
            self.announce_priority(0)

if __name__ == "__main__":
    player = PyGamePlayer(0.2, 1.0, 0.8, frequency=16000)

    buf = StringIO.StringIO("sfdfgdfg")
    s = mixer.Sound(buf)
    player.play_sound(s)


