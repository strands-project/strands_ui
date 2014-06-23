#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import strands_webserver.msg
import strands_webserver.client_utils


class Webloader(object):
    # create messages that are used to publish feedback/result

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.display_no = rospy.get_param("~display", 0)
        self.preempt = False

        # Starting server
        rospy.loginfo("Starting action server")
        self._page_as = actionlib.SimpleActionServer(
            self._action_name + '/page',
            strands_webserver.msg.WebloaderPageAction,
            execute_cb=self.pageGoal,
            auto_start=False
        )
        self._page_as.start()
        self._tmppage_as = actionlib.SimpleActionServer(
            self._action_name + '/tmppage',
            strands_webserver.msg.WebloaderTmpPageAction,
            execute_cb=self.tmppageGoal,
            auto_start=False
        )
        self._tmppage_as.start()
        self._back_as = actionlib.SimpleActionServer(
            self._action_name + '/back',
            strands_webserver.msg.WebloaderBackAction,
            execute_cb=self.backGoal,
            auto_start=False
        )
        self._tmppage_as.register_preempt_callback(self.preemptTmpPage)
        self._back_as.start()
        self._reload_as = actionlib.SimpleActionServer(
            self._action_name + '/reload',
            strands_webserver.msg.WebloaderReloadAction,
            execute_cb=self.reloadGoal,
            auto_start=False
        )
        self._reload_as.start()
        rospy.loginfo("...done.")

        self.history = []

    def pageGoal(self, goal):
        if goal.relative:
            self.display_relative(goal.page)
            self.history.append({"relative": True, "page": goal.page})
        else:
            self.display_content(goal.page)
            self.history.append({"relative": False, "page": goal.page})
        self._page_as.set_succeeded()

    def tmppageGoal(self, goal):
        self.preempt = False
        self.end_time = rospy.get_time() + goal.timeout
        if goal.relative:
            self.display_relative(goal.page)
        else:
            self.display_content(goal.page)

        while rospy.get_time() < self.end_time:
            rospy.Rate(1).sleep()

        self.display_last()
        if not self.preempt:
            self._tmppage_as.set_succeeded()

    def preemptTmpPage(self):
        self.preempt = True
        self.end_time = 0
        self._tmppage_as.set_preempted()

    def reloadGoal(self, goal):
        if len(self.history) == 0:
            self._reload_as.set_abborted()

        self.display_last()
        self._reload_as.set_succeeded()

    def backGoal(self, goal):
        if len(self.history) == 0:
            self._back_as.set_abborted()

        self.history.pop()
        self.display_last()
        self._back_as.set_succeeded()

    def display_relative(self, page):
        strands_webserver.client_utils.display_relative_page(
            self.display_no,
            page
        )

    def display_last(self):
        if self.history[-1]["relative"]:
            self.display_relative(self.history[-1]["page"])
        else:
            self.display_content(self.history[-1]["page"])

    def display_content(self, page):
        strands_webserver.client_utils.display_content(
            self.display_no,
            page
        )

if __name__ == '__main__':
    rospy.init_node('webloader')
    Webloader(rospy.get_name())
    rospy.spin()
