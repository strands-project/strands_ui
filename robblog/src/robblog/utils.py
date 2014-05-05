import os
import subprocess 
import roslib
import shutil

def which(program):
    """ Get the path for an executable: http://stackoverflow.com/questions/377017/test-if-executable-exists-in-python/377028#377028 """
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None

def init_blog(path):
    """ If path does not exist, create it and init jekyll here """
    if not os.path.isdir(path):
        os.makedirs(path)
        jekyll = which('jekyll')
        if jekyll == None:
            raise Exception('jekyll is not in your path. See README.md for installation instructions.')

        proc = subprocess.Popen([jekyll, 'new', 'robblog'], cwd=path)
        proc.wait()


        path = path + '/robblog'
 
        # now put in default files
        data_path = roslib.packages.get_pkg_dir('robblog') + '/data'
        shutil.copy(data_path + '/_config.yml', path)
        shutil.copy(data_path + '/default.html', path + '/_layouts')

        # and delete the post created by the install
        filelist = os.listdir(path + '/_posts')
        for f in filelist:
            os.remove(path + '/_posts/' + f)

def serve(path, host, port):
    """ Starts jekyll server, return Popen process its runnig in. """
    if not os.path.isdir(path):
        raise Exception('Blog path %s does not exist. Run init_blog with this path first.')
    
    path = path + '/robblog'

    jekyll = which('jekyll')
    if jekyll == None:
        raise Exception('jekyll is not in your path. See README.md for installation instructions.')

    return subprocess.Popen([jekyll, 'serve', '--watch', '--host', host, '--port', port], cwd=path)

