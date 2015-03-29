#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import *
import actionlib
from music_player.srv import *
from pygame_managed_player.pygame_player import PyGamePlayer
from mongodb_media_server import MediaClient

from os import listdir
from os.path import isfile, join, splitext, exists, expanduser
from os import makedirs

import pygame
import roslib

class MusicPlayerServer(object):
    def __init__(self):
        rospy.init_node('music_player_server')
        self.music_set = rospy.get_param('~music_set', 'music_player')
        self.audio_folder = join(expanduser('~'), '.ros', 'music_player')

        self.audio_priority = rospy.get_param('~priority', 0.5)
        self.song_index = 0
        #pygame.mixer.init()
        self.player = PyGamePlayer(0.2, 1.0, self.audio_priority, frequency=44100, stop_callback=self.stopped_cb)

        hostname = rospy.get_param('mongodb_host')
        port = rospy.get_param('mongodb_port')
        self.mc = MediaClient(hostname, port)

        sets = self.mc.get_sets("Music")
        object_id = None
        for s in sets:
            if s[0] == self.music_set:
                object_id = s[2]

        if object_id is None:
            rospy.logwarn('Could not find any set in database matching music_set')
            return

        file_set = self.mc.get_set(object_id)

        if len(file_set) == 0:
            rospy.logwarn('No songs available in set ' + self.music_set)

        for f in file_set:
            print "Media name:", f[0]

        if not exists(self.audio_folder):
            makedirs(self.audio_folder)

        for f in file_set:
            file = self.mc.get_media(str(f[2]))
            outfile = open(join(self.audio_folder, f[0]), 'wb')
            filestr = file.read()
            outfile.write(filestr)
            outfile.close()

        self.file_names = [join(self.audio_folder, f[0]) for f in file_set]

        self.paused = True
        self.service = rospy.Service('music_player_service', MusicPlayerService, self.pressed_button)
        while not rospy.is_shutdown():
            #if not self.paused and not pygame.mixer.music.get_busy():
            #    self.song_index = self.song_index + 1
            #    if self.song_index >= len(self.files):
            #        self.song_index = 0
            #    pygame.mixer.music.load(self.files[self.song_index])
            #    pygame.mixer.music.play()
            rospy.sleep(1)

    def stopped_cb(self):
        # playback stopped!
        self.song_index = self.song_index + 1
        if self.song_index >= len(self.file_names):
            self.song_index = 0
        self.player.play_music(self.file_names[self.song_index], blocking=False)
        self.paused = False

    def pressed_button(self, req):
        if req.player_action == MusicPlayerServiceRequest.PLAY:
            #pygame.mixer.music.load(self.files[self.song_index])
            #pygame.mixer.music.play()
            self.player.play_music(self.file_names[self.song_index], blocking=False)
            self.paused = False
        elif req.player_action == MusicPlayerServiceRequest.PAUSE:
            self.paused = True
            #pygame.mixer.music.stop()
            self.player.pause()
        elif req.player_action == MusicPlayerServiceRequest.PREVIOUS:
            #pygame.mixer.music.stop()
            self.song_index = self.song_index - 1
            if self.song_index < 0:
                self.song_index = len(self.file_names)-1
            #pygame.mixer.music.load(self.files[self.song_index])
            #pygame.mixer.music.play()
            self.player.play_music(self.file_names[self.song_index], blocking=False)
            self.paused = False
        elif req.player_action == MusicPlayerServiceRequest.NEXT:
            print self.file_names[self.song_index]
            #pygame.mixer.music.stop()
            self.song_index = self.song_index + 1
            if self.song_index >= len(self.file_names):
                self.song_index = 0
            print self.file_names[self.song_index]
            #pygame.mixer.music.load(self.files[self.song_index])
            #pygame.mixer.music.play()
            self.player.play_music(self.file_names[self.song_index], blocking=False)
            self.paused = False
        else:
            rospy.logwarn('Service argument has to be 0-3: PLAY, PAUSE, NEXT, PREVIOUS')

        return MusicPlayerServiceResponse(self.file_names[self.song_index], self.audio_priority)


if __name__ == '__main__':
    server = MusicPlayerServer()
