#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import *
import actionlib
from aaf_music_player.srv import *
from pygame_managed_player.pygame_player import PyGamePlayer

from os import listdir
from os.path import isfile, join, splitext

import pygame
import roslib

class AAFMusicPlayerServer(object):
    def __init__(self):
        rospy.init_node('aaf_music_player_server')
        folder = rospy.get_param('~folder', '')
        if len(folder) == 0:
            rospy.logwarn('No media folder provided!')
            return

        self.audio_priority = 0.5
        self.media_path = folder
        print self.media_path
        all_files = [ join(self.media_path, f) for f in listdir(self.media_path) if isfile(join(self.media_path, f))]
        self.files = [ name + ext for (name, ext) in map(lambda f: splitext(f), all_files) if ext == '.mp3' ]
        self.song_index = 0
        #pygame.mixer.init()
        self.player = PyGamePlayer(0.2, 1.0, 0.5, frequency=44100, stop_callback=self.stopped_cb)
        
        self.paused = True
        self.service = rospy.Service('aaf_music_player_service', MusicPlayerService, self.pressed_button)
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
        if self.song_index >= len(self.files):
            self.song_index = 0
        self.player.play_music(self.files[self.song_index], blocking=False)
        self.paused = False       

    def pressed_button(self, req):
        if req.player_action == MusicPlayerServiceRequest.PLAY:
            #pygame.mixer.music.load(self.files[self.song_index])
            #pygame.mixer.music.play()
            self.player.play_music(self.files[self.song_index], blocking=False)
            self.paused = False
        elif req.player_action == MusicPlayerServiceRequest.PAUSE:
            self.paused = True
            #pygame.mixer.music.stop()
            self.player.pause()
        elif req.player_action == MusicPlayerServiceRequest.PREVIOUS:
            #pygame.mixer.music.stop()
            self.song_index = self.song_index - 1
            if self.song_index < 0:
                self.song_index = len(self.files)-1
            #pygame.mixer.music.load(self.files[self.song_index])
            #pygame.mixer.music.play()
            self.player.play_music(self.files[self.song_index], blocking=False)
            self.paused = False
        elif req.player_action == MusicPlayerServiceRequest.NEXT:
            print self.files[self.song_index]
            #pygame.mixer.music.stop()
            self.song_index = self.song_index + 1
            if self.song_index >= len(self.files):
                self.song_index = 0
            print self.files[self.song_index]
            #pygame.mixer.music.load(self.files[self.song_index])
            #pygame.mixer.music.play()
            self.player.play_music(self.files[self.song_index], blocking=False)
            self.paused = False
        else:
            rospy.logwarn('Service argument has to be 0-3: PLAY, PAUSE, NEXT, PREVIOUS')

        return MusicPlayerServiceResponse(self.files[self.song_index], self.audio_priority)
        

if __name__ == '__main__':
    server = AAFMusicPlayerServer()

