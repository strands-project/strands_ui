#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mongodb_media_server import MediaClient
from mongodb_video_player.srv import VideoPlayerService, VideoPlayerServiceRequest, VideoPlayerServiceResponse

from os.path import join, exists, expanduser
from os import makedirs

from threading import Thread


class VideoServer(object):
    def __init__(self, name):
        rospy.loginfo("Creating "+name+" server")
        self.video_folder = join(expanduser('~'), '.ros', 'video_server')
        self.video_set = rospy.get_param('~video_set', 'video_set')
        self.video_pub = rospy.Publisher(
            rospy.get_param("~video_pub", "~video"),
            Image,
            queue_size=2
        )
        self.video_index = 0
        self.rate = rospy.Rate(25)
        self.bridge = CvBridge()
        self.thread = None

        self.mc = MediaClient(
            rospy.get_param("mongodb_host"),
            rospy.get_param("mongodb_port")
        )

        sets = self.mc.get_sets("Video")
        object_id = None
        for s in sets:
            if s[0] == self.video_set:
                object_id = s[2]

        if object_id is None:
            rospy.logwarn('Could not find any set in database matching video_set')
            return

        file_set = self.mc.get_set(object_id)

        if len(file_set) == 0:
            rospy.logwarn('No videos available in set ' + self.video_set)

        for f in file_set:
            print "Media name:", f[0]

        if not exists(self.video_folder):
            makedirs(self.video_folder)

        for f in file_set:
            file = self.mc.get_media(str(f[2]))
            outfile = open(join(self.video_folder, f[0]), 'wb')
            filestr = file.read()
            outfile.write(filestr)
            outfile.close()

        self.file_names = [join(self.video_folder, f[0]) for f in file_set]

        self.service = rospy.Service('video_player_service', VideoPlayerService, self.pressed_button)
        rospy.loginfo(" ... started "+name)

    def pub_video(self, filename):
        try:
            video = cv2.VideoCapture(filename)
        except:
            rospy.logwarn("Video %s not found" % filename)
            return
        if not video.isOpened():
            rospy.logwarn("Capture stream could not be opened")
            return

        fps = rospy.Rate(video.get(cv2.cv.CV_CAP_PROP_FPS))

        ret, frame = video.read()
        while self.play and ret and not rospy.is_shutdown():
            img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.video_pub.publish(img)
            fps.sleep()
            ret, frame = video.read()

    def pressed_button(self, req):
        if req.player_action == VideoPlayerServiceRequest.PLAY:
            self.play = True
            self.thread = Thread(target=self.pub_video, args=(self.file_names[self.video_index],))
            self.thread.start()
            self.paused = False
        elif req.player_action == VideoPlayerServiceRequest.PAUSE:
            self.paused = True
            self.play = False
            if self.thread:
                self.thread.join()
        elif req.player_action == VideoPlayerServiceRequest.PREVIOUS:
            self.play = False
            if self.thread:
                self.thread.join()
            self.video_index = self.video_index - 1
            if self.video_index < 0:
                self.video_index = len(self.file_names)-1
            self.play = True
            self.thread = Thread(target=self.pub_video, args=(self.file_names[self.video_index],))
            self.thread.start()
            self.paused = False
        elif req.player_action == VideoPlayerServiceRequest.NEXT:
            self.play = False
            if self.thread:
                self.thread.join()
            self.video_index = self.video_index + 1
            if self.video_index >= len(self.file_names):
                self.video_index = 0
            self.play = True
            self.thread = Thread(target=self.pub_video, args=(self.file_names[self.video_index],))
            self.thread.start()
            self.paused = False
        else:
            rospy.logwarn('Service argument has to be 0-3: PLAY, PAUSE, NEXT, PREVIOUS')

        return VideoPlayerServiceResponse(self.file_names[self.video_index])

if __name__ == '__main__':
    rospy.init_node('video_server')
    server = VideoServer(rospy.get_name())
    rospy.spin()
