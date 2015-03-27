#!/usr/bin/python

import rospy
import roslib

import web
import pymongo
import gridfs
from bson import json_util, Binary, ObjectId
import os
import Image
import StringIO
import signal

file_set_types = {'Video': ['video/mp4'],
                  'Music': ['audio/mpeg'],
                  'Photo': ['image/jpeg', 'image/png'],}


### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('mongodb_media_server') + '/www'
render = web.template.render(TEMPLATE_DIR, base='base',
                             globals={'set_types': file_set_types})
os.chdir(TEMPLATE_DIR)
# so that the static content can be served from here.

### MongoDB access
mongo = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                            rospy.get_param("mongodb_port"))
filesystem = gridfs.GridFS(mongo.media_server)
filesystem_thumbs = gridfs.GridFS(mongo.media_server, collection='thumbscache')
file_sets = mongo.media_server.filesets

class MediaServer(web.application):
    def __init__(self):
        sets_expression = "/view_sets/(" + "|".join(file_set_types.keys()) + ")"
        urls = (
            '/', 'MasterPage',
            '/file_manager', 'FileManager',
            sets_expression, 'ViewSets',
            '/put_media', 'UploadFile',
            '/get_media/([a-zA-Z0-9]+)', 'DownloadFile',
            '/get_media_by_name/(.+)', 'DownloadFileByName',
            '/get_media_thumb/([a-zA-Z0-9]+)', 'GetFileThumbnail',
            '/delete_media/([a-zA-Z0-9]+)', 'DeleteFile',
            '/create_set/([a-zA-Z0-9]+)', 'CreateSet',
            '/delete_set/([a-zA-Z0-9]+)', 'DeleteSet',
            '/edit_set/([a-zA-Z0-9]+)', 'EditSet',
            '/edit_set/([a-zA-Z0-9]+)/add/([a-zA-Z0-9]+)', 'EditSetAdd',
            '/edit_set/([a-zA-Z0-9]+)/remove/([a-zA-Z0-9]+)', 'EditSetRemove',
        )
        web.application.__init__(self, urls, globals())
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def run(self, port=8027, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', port))
    
    def signal_handler(self, signum, frame):
        self.stop()
        print "mongodb_media_server stopped"

class MasterPage(object):
    def GET(self):
        return render.main_page()

class ViewSets(object):
    def GET(self, set_type):
        sets = []
        for s in  file_sets.find({'set_type': set_type,}):
            sets.append( (s['name'], len(s['items']), s["_id"]) )
        
        return render.sets(sets, set_type)

class CreateSet(object):
    def POST(self, set_type):
        set_name = web.input()['set_name']
        if len(set_name) <  1:
            raise web.seeother(web.ctx.env.get('HTTP_REFERER'))
        existing = file_sets.find_one({'name': set_name,
                                       'set_type': set_type,})
        if existing is not None:
            raise web.seeother(web.ctx.env.get('HTTP_REFERER'))
        print "Creating new set ", set_name
        try:
            _id =  file_sets.insert({'name': set_name,
                              'set_type': set_type,
                              'items': [],} )
            print "SEE OTHER:", '/edit_set/'+str(_id)
            raise web.seeother(web.ctx.env.get('HTTP_REFERER'))
            # TODO: redirect straight to edit of this set; fix content error
            # in firefox.
            # raise web.seeother('/edit_set/%s' % str(_id)) 
        except:
            raise web.seeother(web.ctx.env.get('HTTP_REFERER'))
        

class DeleteSet(object):
    def GET(self, file_id):
        try:
            file_sets.remove(ObjectId(str(file_id)))
        except Exception, e:
            print e
            raise web.notfound()
        raise web.seeother(web.ctx.env.get('HTTP_REFERER'))
    
class EditSet(object):
    def GET(self, set_id):
        s =  file_sets.find_one({'_id': ObjectId(set_id),})
        if s is None:
            raise web.notfound()
        
        available =  set([])
        used =  set([])
        for i in s['items']:
            try:
                f =  filesystem.get(ObjectId(i))
                used.add( (f.filename, f.content_type, f._id) )
            except:
                # maybe the file was remeoved, delete it from the set
                s['items'].remove(i)
                file_sets.save(s)
        # TODO: when migrated to newer gridfs library change to use that
        for f in mongo.media_server['fs.files'].find():
            if f['contentType'] in file_set_types[s['set_type']]:
                available.add( (f['filename'], f['contentType'], f['_id']) )
        available =  available - used
        return render.edit_set(s, available, used)
    
class EditSetAdd(object):
    def GET(self, set_id, file_id):
        s =  file_sets.find_one({'_id': ObjectId(set_id),})
        if s is None:
            raise web.notfound()
        f =  filesystem.get(ObjectId(file_id))
        if f is None:
            raise web.notfound()
        s['items'].append(file_id)
        file_sets.save(s)
        raise web.seeother(web.ctx.env.get('HTTP_REFERER'))
    
class EditSetRemove(object):
    def GET(self, set_id, file_id):
        s =  file_sets.find_one({'_id': ObjectId(set_id),})
        if s is None:
            raise web.notfound()
        f =  filesystem.get(ObjectId(file_id))
        if f is None:
            raise web.notfound()
        s['items'].remove(file_id)
        file_sets.save(s)
        raise web.seeother(web.ctx.env.get('HTTP_REFERER'))

        
class FileManager(object):        
    def GET(self):
        files_list =  []
        for f in filesystem.list():
            file_meta =  filesystem.get_last_version(f)
            files_list.append( ( file_meta._id, f,
                                 file_meta.length, file_meta.content_type) )
        return render.file_manager(files_list)
  
        
class UploadFile(object):
    def POST(self):
        x = web.input(myfile={})
        fname=x['myfile'].filename
        if filesystem.exists({"filename":fname}):
            i=1
            filename_parts = fname.split(".")
            if len(filename_parts)<2:
                fname_numbered = fname+"%d"
            else:
                fname_numbered = ( ".".join(filename_parts[:-1]) + "%d." +
                                   filename_parts[-1] )
            while filesystem.exists({"filename":fname_numbered%i}):
                i=i+1
                
            fname = fname_numbered%i
        filesystem.put(x['myfile'].value,
                       filename=fname,
                       content_type=x['myfile'].type)
        raise web.seeother('/file_manager')
 
class DownloadFile(object):  
    def GET(self, file_id):
        try:
            file_ = filesystem.get(ObjectId(str(file_id)))
        except Exception, e:
            print e
            raise web.notfound()
        
        web.header("Content-Type", file_.content_type)
        return file_.read()
 
class DownloadFileByName(object):  
    def GET(self, file_name):
        try:
            file_ = filesystem.get_last_version(file_name)
        except Exception, e:
            print e
            raise web.notfound()
        
        web.header("Content-Type", file_.content_type)
        return file_.read()
    
class DeleteFile(object):
    def GET(self, file_id):
        try:
            filesystem.delete(ObjectId(str(file_id)))
        except Exception, e:
            print e
            raise web.notfound()
        raise web.seeother('/file_manager')
    
class GetFileThumbnail(object):
    def GET(self, file_id):
        if filesystem_thumbs.exists(ObjectId(str(file_id))):
            print "THUMB EXISTS IN CACHE."
            file_ = filesystem_thumbs.get(ObjectId(str(file_id)))
            web.header("Content-Type", file_.content_type)
            return file_.read()
        else:
            # generate and cash a thumbnail if it is a image type
            file_ = filesystem.get(ObjectId(str(file_id)))
            content_major =  file_.content_type.split('/')[0]
            if content_major == "image":
                # generate a thumbnail
                size = (128,128)
                s =  StringIO.StringIO()
                im = Image.open(file_)
                im.thumbnail(size, Image.ANTIALIAS)
                im.save(s, "PNG")
                s.seek(0)
                print "GENERATED NEW THUMB"
                filesystem_thumbs.put(s.read(), _id=ObjectId(str(file_id)), 
                                      content_type='image/png')
                raise web.seeother("/get_media_thumb/%s" % file_id)
            else:
                # already os.cd'ed into www
                if os.path.isfile("static/file_type_icons/%s.png" % content_major):
                    raise web.seeother("/static/file_type_icons/%s.png" % content_major)
                else:
                    raise web.seeother("/static/file_type_icons/unknown_type.png")
                    
        return file_.read()
    
if __name__ == "__main__":
    rospy.init_node("mongodb_media_server")
    rospy.loginfo("mongodb_media_server web server starting..")
    app =  MediaServer()
    app.run()
