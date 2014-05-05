#!/usr/bin/env python

import rospy
import roslib
from ros_datacentre.message_store import MessageStoreProxy
from robblog.msg import RobblogEntry
import robblog.utils


from datetime import *

if __name__ == '__main__':
    rospy.init_node("robblog_example")

    # Create some blog entries
    msg_store = MessageStoreProxy(collection='example_blog')

    create_entries = False
    if create_entries:
        e1 = RobblogEntry(title='Test Title 1', body='blah blah')
        e2 = RobblogEntry(title='Test Title 2', body='blah blah')
        e3 = RobblogEntry(title='Test Title 3', body='blah blah')
        msg_store.insert(e1)
        msg_store.insert(e2)
        msg_store.insert(e3)

    # where are the blog files going to be put
    blog_path = roslib.packages.get_pkg_dir('robblog') + '/content'
    # initialise blog
    robblog.utils.init_blog(blog_path)
    proc = robblog.utils.serve(blog_path, 'localhost', '4040')

    while not rospy.is_shutdown():
        rospy.sleep(1)

    proc.terminate()

