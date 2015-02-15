import pygame.mixer as mixer
import pygame.event as event
from pygame import init
from pygame import NOEVENT, USEREVENT
import rospy
from strands_ui.msg import SoundPriorities
import threading
from pprint import pformat


ANNOUNCE_TOPIC = '/pygame_player_negotiation'
START_MUSIC = USEREVENT + 1
START_SOUND = USEREVENT + 2
END_EVENT = USEREVENT + 3


class _PygameEvDispatcher(threading.Thread):

    def __init__(self, callback):
        super(_PygameEvDispatcher, self).__init__()
        self.callback = callback

    def run(self):
        while not rospy.is_shutdown():
            e = event.poll()
            if e.type == NOEVENT:
                rospy.sleep(0.1)
                continue
            else:
                if e.type == END_EVENT:
                    self.callback(e)


class PyGamePlayer:
    """
    A PyGame-based Music and Sound Player that
        "negotiates" the volume between different components running instances
        of this player through a dedicated topic.
    """
    def __init__(self, min_vol=0.2, max_vol=1.0, priority=1.0,
                 stop_callback=None, topic=ANNOUNCE_TOPIC,
                 frequency=22050, size=-16, channels=2, id=1):
        """
        Creates a PyGame-based Player and initialises it.

        :param min_vol: the minimum volume when another component has higher
            priority. Set between 0.0 and 1.0.
        :param max_vol: the maximum volume when no other component has higher
            priority. Set between 0.0 and 1.0.
        :param priority: The priority of this player. If this component has
            highest priority of all playing, max_vol is used.
        :param stop_callback: a function() to call when the playing stops.
        :param topic: the topic of type stands_ui/SoundPriorities used to
            negotiate the volume between components (usually leave at default)
        :param frequency: the sampling rate of the player. Needs to match
            the sound's or music's sampling rate.
        :param size: the bit width of the sound.
            Usually size=-16 for signed 16 bits. Needs to match
            the sound's or music's size.
        :param channels: number of channels. Needs to match
            the sound's or music's number of channels.
        :param id: The channels id, usually left at default.

        """

        init()
        mixer.init(frequency, size, channels)
        self.priority_map = {}
        self.min_vol = min_vol
        self.max_vol = max_vol
        self.priority = priority
        self.channel = mixer.Channel(id)
        self.announcer_pub = rospy.Publisher(topic,
                                             SoundPriorities, queue_size=100)
        self.announcer_sub = rospy.Subscriber(topic, SoundPriorities,
                                              self.__priority_update)
        self.dispatcher = _PygameEvDispatcher(self.__finished_playing_cb)
        self.cv = threading.Condition()
        self.playing = END_EVENT
        self.stop_callback = stop_callback
        self.dispatcher.start()

    def __finished_playing_cb(self, e):
        self.cv.acquire()
        self.playing = e.type
        rospy.logdebug('stop event=%d', e.type-USEREVENT)
        self.__announce_priority(0)
        self.cv.notify_all()
        self.cv.release()
        self.stop_callback()

    def __set_volume(self):
        max_key, max_value = max(self.priority_map.iteritems(),
                                 key=lambda x: x[1])
        vol = self.max_vol

        if max_key == rospy.get_name():
            vol = self.max_vol
        else:
            if max_value > self.priority:
                vol = self.min_vol
            else:
                vol = self.max_vol
        rospy.loginfo("setting volume to %f", vol)
        self.channel.set_volume(vol)
        mixer.music.set_volume(vol)

    def __priority_update(self, data):
        self.priority_map[data.id] = data.priority
        rospy.loginfo("updated priority map: %s", pformat(self.priority_map))
        self.__set_volume()

    def __announce_priority(self, prio):
        s = SoundPriorities()
        s.id = rospy.get_name()
        s.priority = prio
        self.announcer_pub.publish(s)

    def __announce_start(self, e):
        self.__announce_priority(self.priority)
        self.cv.acquire()
        self.playing = e
        self.cv.release()
        event.post(event.Event(e))
        rospy.logdebug('start event=%d', e-USEREVENT)

    def __wait_for_end(self):
        self.cv.acquire()
        while not self.playing == END_EVENT:
            self.cv.wait()
        self.cv.release()

    def pause(self):
        """
        pause currently playing music or sound
        """
        if self.playing == START_MUSIC:
            mixer.music.pause()
        if self.playing == START_SOUND:
            self.channel.pause()

    def unpause(self):
        """
        unpause currently playing music or sound
        """
        if self.playing == START_MUSIC:
            mixer.music.unpause()
        if self.playing == START_SOUND:
            self.channel.unpause()

    def play_sound(self, s, blocking=True):
        """
        Play a Pygame Sound object

        :param s: the sound to play
        :param blocking: wait for the sound to finish playing before returning
        """
        rospy.loginfo("start playing sound")
        self.__announce_start(START_SOUND)
        self.channel.play(s)
        self.channel.set_endevent(END_EVENT)
        if blocking:
            self.__wait_for_end()

    def play_music(self, fname, blocking=True):
        """
        Play a music file

        :param fname: path of the music file to play (e.g. wav, mp3, or ogg)
        :param blocking: wait for the music to finish playing before returning
        """
        rospy.loginfo("start playing music from %s", fname)
        mixer.music.load(fname)
        self.__announce_start(START_MUSIC)
        mixer.music.play()
        mixer.music.set_endevent(END_EVENT)
        if blocking:
            self.__wait_for_end()


### test code below:
def cb():
    print "end callback called"

if __name__ == "__main__":
    import sys
    rospy.init_node('testplayer')
    player = PyGamePlayer(0.2, 1.0, float(sys.argv[2]), frequency=44100, stop_callback=cb)
#    buf = StringIO("sfdfgdfg")
#    s = mixer.Sound(buf)
#    player.play_sound(s)
    rospy.sleep(1)
    player.play_music(sys.argv[1], blocking=False)
    print "played"
    rospy.sleep(0.5)
    player.pause()
    rospy.sleep(5)
    player.unpause()

    rospy.spin()
