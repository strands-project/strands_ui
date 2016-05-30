import roslib; roslib.load_manifest("robot_talk")
import rospy
import random
from tabulate import tabulate

from robot_talk.msg import RobotTalk
from actionlib import SimpleActionClient
from mary_tts.msg import maryttsAction, maryttsGoal
from mongodb_store.message_store import MessageStoreProxy


class Pmf(object):

    def __init__(self):
        self.d = dict()

    def prob(self, x):
        if x == -1:
            return 0.0
        return self.d[x]  

    def set(self, x, val):
        self.d[x] = val

    def unset(self, x):
        if x in self.d:
            self.d.pop(x)

    def keys(self):
        return self.d.keys()

    def total(self):
        total = sum(self.d.itervalues())
        return total
        
    def normalize(self):
        total = self.total()
        if total == 0.0:
            return total

        for x in self.d:
            self.d[x] /= total

        return total

    def random(self):
        target = random.random()
        total = 0.0
        for x, p in self.d.iteritems():
            total += p
            if total >= target:
                return x
        return self.keys()[-1]


class RobotTalkProxy():

    def __init__(self, collection="robot_talk"):
        self._msg_store=MessageStoreProxy(database="message_store", collection=collection)

    def _get_next_id(self):
        entries = self._msg_store.query(RobotTalk._type, message_query={})
        max_id = 0
        for e,em in entries:
            if int(e.id) > max_id:
                max_id = int(e.id)
        self._next_id = max_id + 1
        return self._next_id

    def _get_distinct_topics(self):
        res = self._msg_store.query(RobotTalk._type, message_query={})
        topics = {}
        for e, meta in res:
            if e.topic not in topics:
                topics[e.topic] = 1
            else:
                topics[e.topic] += 1
        return topics

    def _get_random_id(self, topic):
        entries =  self._msg_store.query(RobotTalk._type, message_query={'topic': topic})
        if len(entries) == 0:
            print "No entry found for topic:", topic
            return 
            
        pmf = self._generate_pmf(entries)
        talk_id = pmf.random()
        print "Chosen ID: ", talk_id, "- Probability:", pmf.prob(talk_id)
        return talk_id
        
    def _generate_pmf(self, entries):
        pmf = Pmf()
        for e, meta in entries:
            pmf.set(e.id, e.weight)
        pmf.normalize()
        return pmf

    def _print(self, entries):
        table = []
        for e, emeta in entries:
            table.append([e.id, e.topic, e.text, e.weight])
            
        print tabulate(table, headers=['ID', 'Topic', 'Text', 'Weight'], tablefmt='rst')
        print "Total number:", len(entries)

    def add(self, topic, text, weight=1.0):
        talk = RobotTalk()
        talk_id = self._get_next_id()
        talk.id     = str(talk_id)
        talk.topic  = str(topic)
        talk.text   = str(text)
        talk.weight = float(weight)
        print "Adding entry: ID:", talk_id, "- Topic:", topic, "- Text:", text, "- Weight", weight
        _id = self._msg_store.insert(talk)
        return talk.id

    def remove(self, talk_id):
        res = self._msg_store.query(RobotTalk._type, message_query={'id' : talk_id})
        if len(res) == 0:
            print "Entry not found. ID:", talk_id
            print "Remove failed."
            return

        talk_meta = res[0][1]
        _id = talk_meta['_id']
        try:
            print "Removing entry ID:", talk_id
            self._msg_store.delete(str(_id))
        except:
            print "Removal failed."
            return False
        print "Removal successful."
        return True

    def update(self, talk_id, topic, text, weight=1.0):

        res = self._msg_store.query(RobotTalk._type, message_query={'id' : talk_id})
        if len(res) == 0:
            print "Entry not found. ID:", talk_id
            print "Update failed."
            return

        e, meta = res[0]
        talk = RobotTalk()
        talk.id     = str(e.id)
        talk.topic  = str(topic)
        talk.text   = str(text)
        talk.weight = float(weight)
        print "Updating entry ID:", talk_id, "- Topic:", topic, "- Text:", text, "- Weight", weight
        res = self._msg_store.update_id(meta['_id'], talk, meta, upsert = False)
        if res.success == True:
            print "Update successful."
        else:
            print "Update failed."

    def search(self, query):
        res = self._msg_store.query(RobotTalk._type, message_query={'text' : {'$regex' : '.*' + query + '.*'}})
        print "Listing entries for query:", query
        self._print(res)        
        
    def list(self, topic):
        if topic != None:
            print "Listing entries for topic:", topic
            entries = self._msg_store.query(RobotTalk._type, message_query={'topic': topic})
        else:
            print "Listing all entries"
            entries = self._msg_store.query(RobotTalk._type, message_query={})
        self._print(entries)

    def topics(self):
        topics = self._get_distinct_topics()
        table = []
        for t in topics:
            table.append([t, topics[t]])
        print tabulate(table, headers=['Topic', 'No. of entries'],tablefmt='rst')
        print "Total number of topics:", len(topics.keys())
        
        
    def play(self, talk_id):
        res = self._msg_store.query(RobotTalk._type, message_query={'id' : talk_id})
        if len(res) == 0 :
            print "Entry not found. ID:", talk_id
            return
            
        talk = res[0][0]
        print "Now playing:", talk.text
        print
        speaker = SimpleActionClient('/speak', maryttsAction)
        #print "Waiting for /speak server."
        speaker.wait_for_server()
        goal =  maryttsGoal()
        goal.text = talk.text
        res = speaker.send_goal(goal)

    def play_random(self, topic):
        talk_id = self._get_random_id(topic)
        self.play(talk_id)

    def get_random_text(self, topic):
        talk_id = self._get_random_id(topic)
        
        res = self._msg_store.query(RobotTalk._type, message_query={'id' : talk_id})
        if len(res) == 0 :
            print "Entry not found. ID:", talk_id
            return
        talk = res[0][0]
        return talk.text
