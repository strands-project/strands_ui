#!/usr/bin/env python
# -*- coding: utf-8 -*-
import httplib
import urllib

import StringIO

from Queue import *

import logging

from mary_tts.srv import *
import rospy
import roslib

import actionlib
import mary_tts.msg
import os

from pygame_managed_player.pygame_player import PyGamePlayer

import pygame.mixer as mixer


speakQueue = Queue()
replyQueue = Queue()


class RosMary(object):
    def __init__(self, mary_client):
        self.mary_client = mary_client
        rospy.Service('ros_mary', ros_mary, self.speak)
        rospy.Service('ros_mary/set_voice', SetVoice, self.set_voice)
        rospy.Service('ros_mary/set_locale', SetLocale, self.set_locale)
        host = rospy.get_param("~host", "localhost")
        self.mary_client.set_host(host)

        # What voices are available?
        filelist = os.listdir(os.path.join(roslib.packages.get_pkg_dir("mary_tts"),
                                           "marytts-5.0/lib/"))
        print "Ready to speak."
        print "Locales:"
        self._locales = []
        for i in filelist:
            if "lang" in i:
                lang = i[13:i.find("-5.0")]
                if lang == 'en':  # English special case with two locales in one
                    lang = "en_GB"
                    print " - ", lang
                    self._locales.append(lang)
                    lang = "en_US"
                    print " - ", lang
                    self._locales.append(lang)
                else:
                    print " - ", lang
                    self._locales.append(lang)
        print "Voices: "
        self._voices = []
        for i in filelist:
            if "voice" in i and not "voices" in i:
                voice = i[6:i.find("-5.0")]
                print " - ", voice
                self._voices.append(voice)
        voice = rospy.get_param("~voice")
        locale = rospy.get_param("~locale")
        if locale not in self._locales:
            rospy.logwarn("Selected voice '%s' not available, using default!" % locale)
        else:
            self.mary_client.locale = locale
            print "Selected locale:", self.mary_client.locale
        if voice not in self._voices:
            rospy.logwarn("Selected voice '%s' not available, using default!" % voice)
        else:
            self.mary_client.voice = voice
            print "Selected voice:", self.mary_client.voice

    def speak(self, req):
        """ Speak service handler """
        if req.text == '':
            rospy.logwarn("mary_tts was asked to produce an empty string.")
            return False
        speakQueue.put(req.text)
        return True

    def set_voice(self, req):
        """ Voice setting service handle """
        if not req.voice_name in self._voices:
            rospy.logwarn("Trying to set voice to unknown:  %s", req.voice_name)
            return False
        self.mary_client.set_voice(req.voice_name)
        rospy.loginfo("Set voice to %s", self.mary_client.get_voice())
        return True

    def set_locale(self, req):
        """ Locale setting service handle """
        if not req.locale_name in self._locales:
            rospy.logwarn("Trying to set locale to unknown:  %s", req.locale_name)
            return False
        self.mary_client.set_locale(req.locale_name)
        rospy.loginfo("Set locale to %s", self.mary_client.get_locale())
        return True


#
# A simple MARY TTS client in Python, using pulseaudio for playback
#
# based on Code from Hugh Sasse (maryclient-http.py)
#
# 2013 by G. Bartsch. License: LGPLv3
class maryclient:

    def __init__(self):

        self.host = "127.0.0.1"
        self.port = 59125
        self.input_type = "TEXT"
        self.output_type = "AUDIO"
        self.audio = "WAVE_FILE"
        self.locale = "en_US"
        self.voice = "cmu-bdl-hsmm"
        self._action_name = "speak"
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                mary_tts.msg.maryttsAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()

    def set_host(self, a_host):
        """Set the host for the TTS server."""
        self.host = a_host

    def get_host(self):
        """Get the host for the TTS server."""
        return self.host

    def set_port(self, a_port):
        """Set the port for the TTS server."""
        self.port = a_port

    def get_port(self):
        """Get the port for the TTS server."""
        return self.port

    def set_input_type(self, t):
        """Set the type of input being
           supplied to the TTS server
           (such as 'TEXT')."""
        self.input_type = t

    def get_input_type(self):
        """Get the type of input being
           supplied to the TTS server
           (such as 'TEXT')."""
        return self.input_type

    def set_output_type(self, t):
        """Set the type of input being
           supplied to the TTS server
           (such as 'AUDIO')."""
        self.output_type = t

    def get_output_type(self):
        """Get the type of input being
           supplied to the TTS server
           (such as "AUDIO")."""
        return self.output_type

    def set_locale(self, a_locale):
        """Set the locale
           (such as "en_US")."""
        self.locale = a_locale

    def get_locale(self):
        """Get the locale
           (such as "en_US")."""
        return self.locale

    def set_audio(self, audio_type):
        """Set the audio type for playback
           (such as "WAVE_FILE")."""
        self.audio = audio_type

    def get_audio(self):
        """Get the audio type for playback
           (such as "WAVE_FILE")."""
        return self.audio

    def set_voice(self, a_voice):
        """Set the voice to speak with
           (such as "dfki-prudence-hsmm")."""
        self.voice = a_voice

    def get_voice(self):
        """Get the voice to speak with
           (such as "dfki-prudence-hsmm")."""
        return self.voice

    def generate(self, message):
        """Given a message in message,
           return a response in the appropriate
           format."""
        raw_params = {"INPUT_TEXT": message,
                "INPUT_TYPE": self.input_type,
                "OUTPUT_TYPE": self.output_type,
                "LOCALE": self.locale,
                "AUDIO": self.audio,
                "VOICE": self.voice,
                }
        params = urllib.urlencode(raw_params)
        headers = {}

        # Open connection to self.host, self.port.
        conn = httplib.HTTPConnection(self.host, self.port)

        # conn.set_debuglevel(5)

        conn.request("POST", "/process", params, headers)
        logging.warning('Watch out!' + params)

        response = conn.getresponse()
        if response.status != 200:
            print response.getheaders()
            raise RuntimeError("{0}: {1}".format(response.status,
                               response.reason))
        return response.read()

    def execute_cb(self, goal):
        # helper variables
        rospy.loginfo("action triggered: say %s", goal.text)
        if goal.text == '':
            rospy.logwarn("mary_tts was asked to produce an empty string.")
            self._as.set_aborted()
            return
        speakQueue.put(goal.text)
        try:
            replyQueue.get(True, 10)
        except Empty:
            rospy.logwarn("mary speach action failed; maybe took too long (more than 10 seconds), maybe pulse is broke.")
            self._as.set_succeeded(False)
            return
        rospy.loginfo("finished speaking...")
        self._as.set_succeeded()


if __name__ == "__main__":
    rospy.init_node('mary_tts')

    client = maryclient()

    client.set_locale("en_GB")
    client.set_voice("dfki-prudence-hsmm")

    rosmary = RosMary(client)

    player = PyGamePlayer(min_vol=1.0, max_vol=1.0, priority=1.0,
                          frequency=16000)

    while not rospy.is_shutdown():
        try:
            req = speakQueue.get(True, 1)
            rospy.loginfo("say " + req)
            the_sound = mixer.Sound(StringIO.StringIO(client.generate(req)))
            player.play_sound(the_sound, blocking=True)
            rospy.loginfo("played")
            replyQueue.put(True)
        except Empty:
            rospy.logdebug('nothing sent')
