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

    create_entries = True
    if create_entries:
        e1 = RobblogEntry(title='Test Title 1', body='blah blah')
        e2 = RobblogEntry(title='Test Title 2', body='blah blah')
        e3 = RobblogEntry(title='Test Title 3', body='blah blah')
        msg_store.insert(e1)
        msg_store.insert(e2)
        msg_store.insert(e3)
        with open(roslib.packages.get_pkg_dir('robblog') + '/data/example.md' , 'r') as f:
            e4 = RobblogEntry(title='Markdown Example', body=f.read())
            msg_store.insert(e4)
            print e4

    # where are the blog files going to be put
    blog_path = roslib.packages.get_pkg_dir('robblog') + '/content'
    # initialise blog
    robblog.utils.init_blog(blog_path)
    proc = robblog.utils.serve(blog_path, 'localhost', '4040')

    converter = robblog.utils.EntryConverter(blog_path=blog_path, collection='example_blog')
    converter.convert()

    while not rospy.is_shutdown():
        rospy.sleep(1)

    proc.terminate()

