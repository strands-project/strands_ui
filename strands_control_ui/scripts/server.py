#!/usr/bin/python

import rospy
import roslib

import web
import signal
from os import chdir
from os.path import join
from yaml import load, dump
from json import loads as json_load
from strands_control_ui.srv import DemandTask
from strands_control_ui.srv import DemandTaskResponse

#from strands_executive_msgs.srv import DemandTask as SchedulerDemandTask
#from strands_executive_msgs.srv import DemandTaskRequest as SchedulerDemandTaskRequest

### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('strands_control_ui') + '/www'
CFG_DIR = roslib.packages.get_pkg_dir('strands_control_ui') + '/cfg'
WEBTOOLS_DIR = roslib.packages.get_pkg_dir('strands_webtools')


html_config = {
    'rosws_suffix': ':9090',
    'mjpeg_suffix': ':8181',
    'rosws_protocol': 'ws',
    'host': 'localhost'
}

LC = {}
config = {}


render = web.template.render(TEMPLATE_DIR, base='base', globals=globals())
chdir(TEMPLATE_DIR)



class ControlServer(web.application):
    def __init__(self):
        urls = (
            '/', 'DashboardPage',
            '/tasks', 'TasksPage',
            '/setup', 'SetupPage',
            '/admin', 'AdminPage',
            '/webtools/(.*)', 'Webtools'
        )
        web.application.__init__(self, urls, globals())
        rospy.Service(rospy.get_name()+'/demand_task', DemandTask, self.demand_task)
        signal.signal(signal.SIGINT, self.signal_handler)
        self.demand_priority = 100

    def run(self, port=8027, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', port))

    def signal_handler(self, signum, frame):
        self.stop()
        print "aaf_control_server stopped."

    def demand_task(self, req):
        from strands_executive_msgs.srv import CreateTask
        from strands_executive_msgs.msg import Task
        from strands_executive_msgs.srv import AddTasks
        from strands_executive_msgs.srv import AddTasksRequest

        factory_name = '/' + req.action + "_create"
        start_after = rospy.Time.now()+rospy.Duration(secs=30)
        end_before = start_after + rospy.Duration(secs=req.duration)
        # prepare the YAML
        y = {
            "start_after": {"secs": start_after.secs, "nsecs": start_after.nsecs},
            "end_before": {"secs": end_before.secs, "nsecs": end_before.nsecs},
            "start_node_id": req.waypoint,
            "end_node_id": req.waypoint,
        }


        yaml = dump(y) 

        rospy.loginfo("calling with pre-populated yaml: %s" % yaml)

        t = Task()
        try:
            factory = rospy.ServiceProxy(factory_name, CreateTask)
            t = factory.call(yaml).task
            rospy.loginfo("got the task back: %s" % str(t))
        except Exception as e:
            rospy.logwarn("Couldn't instantiate task from factory %s."
                          "error: %s."
                          "This is an error." %
                          (factory_name, str(e)))
            t.start_after = start_after
            t.end_before = end_before
            t.action = req.action

        # use maximum duration of the one given here and the one returned from the constructor
        t.max_duration.secs = max(t.max_duration.secs, req.duration-30)
        t.max_duration.nsecs = 0
        t.start_node_id = req.waypoint
        t.end_node_id = req.waypoint

        # add parameters
        from strands_executive_msgs import task_utils
        params = json_load(req.parms_json)
        
        print params
        for p in params:
            for k, v in p.items():
                rospy.loginfo('add %s parameter: %s' % (k, v)) 
                if (k == "string"):
                    task_utils.add_string_argument(t, v)
                elif (k == "int"):
                    task_utils.add_int_argument(t, int(v))
                elif (k == "float"):
                    task_utils.add_float_argument(t, float(v))

        t.priority = self.demand_priority
        tasks = [t]
        rospy.loginfo('add task %s to schedule now' % t)
        dt = rospy.ServiceProxy('/task_executor/add_tasks', AddTasks)
        try:
            rospy.loginfo(dt(tasks))
        except Exception as e:
            rospy.logerr("Couldn't add task to scheduler. "
                         "error: %s." % str(e))
            t = Task()
            t.action = req.action
        return DemandTaskResponse()

def set_ws_protocol():
    forward =  web.ctx.env.get('HTTP_X_FORWARDED_HOST','')
    if 'lcas.lincoln.ac.uk' in forward:
        html_config['rosws_protocol'] = 'wss'
    else:
        html_config['rosws_protocol'] = 'ws'
    print html_config['rosws_protocol']



class DashboardPage(object):
    def GET(self):
        set_ws_protocol()
        s = web.ctx.homedomain.split(':')
        print s
        html_config['host'] = s[0] + ':' + s[1]
        return render.dashboard()


class TasksPage(object):
    def GET(self):
        set_ws_protocol()
        return render.tasks()


class SetupPage(object):
    def GET(self):
        set_ws_protocol()
        return render.setup()


class HelpPage(object):
    def GET(self):
        set_ws_protocol()
        return render.help()


class AdminPage(object):
    def GET(self):
        set_ws_protocol()
        return render.admin()


class Webtools(object):
    """
    proxies all requests to strands_webtools
    """
    def GET(self, f):
        try:
            p = join(WEBTOOLS_DIR, f)
            rospy.logdebug("trying to serve %s from %s", f, p)
            if f.endswith('.js'):
                web.header('Content-Type', 'text/javascript')
            return open(p, 'r').read()
        except:
            web.application.notfound(app)  # file not found


if __name__ == "__main__":
    rospy.init_node("strands_control_ui_server")
    port = rospy.get_param('~port', 8127)
    html_config['rosws_suffix'] = rospy.get_param('~rosws_suffix', ":9090")
    html_config['mjpeg_suffix'] = rospy.get_param('~mjpeg_suffix', ":8181")
    html_config['rosws_protocol'] = rospy.get_param('~rosws_protocol', "ws")

    config_file = rospy.get_param('~config_file', join(CFG_DIR, 'config.yaml'))
    try:
        with open(config_file, 'r') as stream:
            l = load(stream)
            config.update(l)
            print config
    except:
        rospy.logerr('failed to load config file from %s' % lf)
        raise

    lang = rospy.get_param('~lang', "EN").lower()
    lf = join(CFG_DIR, lang+'.yaml')
    try:
        with open(lf, 'r') as stream:
            l = load(stream)
            LC.update(l)
    except:
        rospy.logerr('failed to load language config from %s' % lf)
        raise
    rospy.loginfo("strands_control_ui_server started.")
    app = ControlServer()
    app.run(port=port)
