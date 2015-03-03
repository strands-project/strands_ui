import pymongo
import gridfs
from bson import json_util, Binary, ObjectId

class MediaClient(object):
    def __init__(self, mongodb_host='localhost', mongodb_port=62345):
        self._mongo = pymongo.MongoClient(mongodb_host, mongodb_port)
        self._filesystem = gridfs.GridFS(self._mongo.media_server)
        self._filesystem_thumbs = gridfs.GridFS(self._mongo.media_server,
                                                collection='thumbscache')
        self._file_sets = self._mongo.media_server.filesets
        
    def get_sets(self, set_type=""):
        """
        Returns a list of file sets. Optionally only of the given set_type
        (Photo|Music|Video). Return is a list of tuples:
        (set_name, number_items, set_id)
        """
        search = {}
        sets = []
        if set_type !=  "":
            search["set_type"] = set_type
        for s in  self. _file_sets.find(search):
            sets.append( (s['name'], len(s['items']), s["_id"]) )
        return sets
    
    def get_set(self, set_id, set_type_name=""):
        """
        Returns a list of the items in a given set. Set selection either by
        set_id if set_type_name is not filled, otherwise by set_type_name.
        set_type_name should be "(Photo|Video|Music)/NAME".
        """
        s =  self._file_sets.find_one({'_id': ObjectId(set_id),})
        if s is None:
            raise Exception("Set id does not exist.")
        items = []
        for i in s['items']:
            try:
                f =  self._filesystem.get(ObjectId(i))
                items.append( (f.filename, f.content_type, f._id) )
            except Exception, e:
                # maybe the file was remeoved, delete it from the set
                # TODO: be less aggressive
                s['items'].remove(i)
                self._file_sets.save(s)
        return items
    
    def get_media(self, media_id):
        """
        Retrieves the media specified by id. Returns a file like object.
        Example:
        
        f=get_media("ssd439fsk2")
        content=f.read()
        content_type=f.content_type
        """
        try:
            file_ = self._filesystem.get(ObjectId(str(media_id)))
        except Exception, e:
            print e
            raise Exception("Can't get file.")
                
        return file_
    
if __name__ == '__main__':
    ''' Main Program '''
    # If this script is executed, run some development tests
    import Image
    mc = MediaClient('localhost', 62345)
    
    for s in mc.get_sets():
        print "SET '%s'" % s[0]
        for i in mc.get_set(s[2]):
            print " - > ITEM %s" % i[0]
            if i[1].startswith("image"):
                f = mc.get_media(i[2])
                im = Image.open(f)
                im.show()