import pygame.mixer as mixer
import pygame.event as event
from pygame import init
from pygame import NOEVENT, USEREVENT
import rospy
from StringIO import StringIO
from strands_ui.msg import SoundPriorities
from std_msgs.msg import String
import threading
from time import sleep
from pprint import pprint


ANNOUNCE_TOPIC = '/pygame_player_negotiation'


class PygameEvDispatcher(threading.Thread):

    def __init__(self, callback):
        super(PygameEvDispatcher, self).__init__()
        self.callback = callback

    def run(self):
        while not rospy.is_shutdown():
            e = event.poll()
            if e.type == NOEVENT:
                rospy.sleep(0.1)
                continue
            else:
                self.callback(e)


class PyGamePlayer:

    def __init__(self, min_vol, max_vol, priority,
                 frequency=22050, size=-16, channels=2, id=1):
        init()
        mixer.init(frequency, size, channels)
        self.priority_map = {}
        self.min_vol = min_vol
        self.max_vol = max_vol
        self.priority = priority
        self.channel = mixer.Channel(id)
        self.announcer_pub = rospy.Publisher(ANNOUNCE_TOPIC,
                                             SoundPriorities, queue_size=100)
        #self.announcer_sub = rospy.Subscriber(ANNOUNCE_TOPIC, SoundPriorities,
        #                                      self.announced)
        self.dispatcher = PygameEvDispatcher(self.finished_playing_cb)
        self.dispatcher.start()
        self.cv = threading.Condition()
        self.playing = False


    def finished_playing_cb(self, e):
        self.cv.acquire()
        self.playing = False
        self.announce_priority(0)
        self.cv.release()


    def announced(self, data):
        self.priority_map[data.id] = data.priority

        max_key, max_value = max(self.priority_map.iteritems(),
                                 key=lambda x: x[1])

        if max_key == rospy.get_name():
            self.channel.set_volume(self.max_vol)
            pygame.mixer.music.set_volume(self.max_vol)
        else:
            if max_value > self.priority:
                self.channel.set_volume(self.min_vol)
                pygame.mixer.music.set_volume(self.min_vol)
            else:
                self.channel.set_volume(self.max_vol)
                pygame.mixer.music.set_volume(self.max_vol)

    def announce_priority(self, prio):
        s = SoundPriorities()
        s.id = rospy.get_name()
        s.priority = prio
        pprint(s)
        self.announcer_pub.publish(s)

    def play_sound(self, s):
        self.announce_priority(self.priority)
        self.channel.play(s)


    def play_music(self, fname, blocking=True):
        self.announce_priority(self.priority)
        mixer.music.load(fname)
        self.playing = True
        mixer.music.play()
        mixer.music.set_endevent(USEREVENT)
        if blocking:
            # Consume one item
            self.cv.acquire()
            while self.playing:
                self.cv.wait()
            self.cv.release()



if __name__ == "__main__":
    rospy.init_node('testplayer')
    player = PyGamePlayer(0.2, 1.0, 0.8, frequency=44100)
#    buf = StringIO("sfdfgdfg")
#    s = mixer.Sound(buf)
#    player.play_sound(s)
    while not rospy.is_shutdown():
        player.play_music('/Volumes/users/Library/Application Support/GarageBand/Instrument Library/Sampler/Sampler Files/Grand Piano/095_B5KM56_S.wav')
        print "played"
        rospy.sleep(10)

    #rospy.spin()
