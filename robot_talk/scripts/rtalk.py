#!/usr/bin/env python
import roslib; roslib.load_manifest("robot_talk")
import rospy
import sys
import argparse

from robot_talk.proxy import RobotTalkProxy
                                                    
if __name__=="__main__":
    rospy.init_node("rtalk")

    parser = argparse.ArgumentParser(prog='rtalk.py')
    subparsers = parser.add_subparsers(help='sub-command -h|--help', dest='subparser_name')
    
    p_add = subparsers.add_parser('add', help='add -h|--help')
    p_add.add_argument("topic", nargs=1, action='store', help='topic of the text output')
    p_add.add_argument("text",   nargs=1, action='store',  help='text output')
    p_add.add_argument("weight", nargs=1, action='store', help='weight for text within topic')

    p_remove = subparsers.add_parser('remove', help='remove -h|--help')
    p_remove.add_argument("id", nargs=1, action='store', help='Id of entry that should be removed')

    p_update = subparsers.add_parser('update', help='update -h|--help')
    p_update.add_argument("id", nargs=1, action='store', help='Id of entry that be should update')
    p_update.add_argument("topic", nargs=1, action='store', help='topic of the text output')
    p_update.add_argument("text",   nargs=1, action='store',  help='text output')
    p_update.add_argument("weight",  nargs=1, action='store', help='weight for text within topic')

    p_list = subparsers.add_parser('list', help='list -h|--help')
    p_list.add_argument("topic", nargs='?', action='store', help='only list entries of topic, otherwise all')

    p_search = subparsers.add_parser('search', help='search -h|--help')
    p_search.add_argument("query", nargs=1, action='store', help='Query string for searching text (not topic)')

    p_play = subparsers.add_parser('play', help='play -h|--help')
    p_play.add_argument("id", nargs=1, action='store', help='Id of entry that should be played')

    p_play_random = subparsers.add_parser('play_random', help='play_random -h|--help')
    p_play_random.add_argument("topic", nargs=1, action='store', help='Play a random entry from topic')

    p_topics = subparsers.add_parser('topics', help='topics -h|--help')
    
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])    
    cmd =  args.subparser_name

    rtalk = RobotTalkProxy("robot_talk")
    print
    if cmd == 'add':
        rtalk.add(args.topic[0], args.text[0], args.weight[0])
    elif cmd == 'remove':
        rtalk.remove(args.id[0])                
    elif cmd == 'update':
        rtalk.update(args.id[0], args.topic[0], args.text[0], args.weight[0])
    elif cmd == 'list':
        if args.topic == None:
            rtalk.list(None)
        else:
            rtalk.list(args.topic)
    elif cmd == 'topics':
        rtalk.topics()
    elif cmd == 'search':
        rtalk.search(args.query[0])
    elif cmd == 'play':
        rtalk.play(args.id[0])
        print "Couldn't hear anything? Is MaryTTS running? Have you checked your volume?"
    elif cmd == 'play_random':
        rtalk.play_random(args.topic[0])
        print "Couldn't hear anything? Is MaryTTS running? Have you checked your volume?"
    else:
        print "Unknown command. Please run 'rtalk.py -h' for more details."

