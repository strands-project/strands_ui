#!/usr/bin/env python

import rospy
from sound_player_server.srv import *
from mongodb_media_server import MediaClient
from pygame_managed_player.pygame_player import PyGamePlayer

from os import listdir
from os.path import isfile, join, splitext, exists, expanduser
from os import makedirs

import pygame
import roslib

class SoundPlayerServer(object):
    def __init__(self):
        rospy.init_node('sound_player_server')
        self.music_set = rospy.get_param('~music_set', 'music_player')
        self.audio_folder = join(expanduser('~'), '.ros', 'sound_player_server')

        self.audio_priority = rospy.get_param('~audio_priority', 0.5)
        self.song_index = 0
        self.player = PyGamePlayer(
            rospy.get_param('~min_volume', 0.2),
            rospy.get_param('~max_volume', 1.0),
            self.audio_priority,
            frequency=44100,
            stop_callback=None
        )

        hostname = rospy.get_param('mongodb_host')
        port = rospy.get_param('mongodb_port')
        self.mc = MediaClient(hostname, port)

        sets = self.mc.get_sets("Music")
        object_id = None
        for s in sets:
            if s[0] == self.music_set:
                object_id = s[2]

        if object_id is None:
            rospy.logwarn('Could not find any set in database matching ' + self.music_set + ' collection.')
            return

        file_set = self.mc.get_set(object_id)

        if len(file_set) == 0:
            rospy.logwarn('No songs available in set ' + self.music_set)
            return

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

        self.service = rospy.Service('sound_player_service', PlaySoundService, self.pressed_button)

        while not rospy.is_shutdown():
            rospy.sleep(1)

    def pressed_button(self, req):
        if len(req.filename) == 0:
            rospy.logwarn('Please provide service with file in ' + self.music_set + ' collection.')
            return PlaySoundServiceResponse(False, 0.0)
        filepath = join(self.audio_folder, req.filename)
        if not isfile(filepath):
            rospy.logwarn(req.filename + ' is not a file in the ' + self.music_set + ' collection.')
            return PlaySoundServiceResponse(False, 0.0)

        self.player.play_music(filepath, blocking=False)

        return PlaySoundServiceResponse(True, self.audio_priority)


if __name__ == '__main__':
    server = SoundPlayerServer()
