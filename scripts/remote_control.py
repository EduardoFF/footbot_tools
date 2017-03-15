import math
from threading import Thread
import subprocess
import time
import fabric
from fabric.state import connections
from fabric.api import env,run,execute,hosts
from fabric import tasks
from fabric.network import ssh
import datetime
ssh.util.log_to_file("paramiko.log", 10)
import paramiko
import socket

def checkhostalive(host):
    ret = subprocess.call("ping -c 1 -W 2 %s" % host,
        shell=True,
        stdout=open('/dev/null', 'w'),
        stderr=subprocess.STDOUT)
    if ret == 0:
        return True
    else:
        print "not alive ",host
        return False

def _is_host_up(host, port):
    # Set the timeout
    original_timeout = socket.getdefaulttimeout()
    new_timeout = 3
    socket.setdefaulttimeout(new_timeout)
    host_status = False
    try:
        transport = paramiko.Transport((host, port))
        host_status = True
    except:
        print('***Warning*** Host {host} on port {port} is down.'.format(
            host=host, port=port)
        )
    socket.setdefaulttimeout(original_timeout)
    return host_status


class RemoteCtrl():
    def __init__(self, robots=[]):
        self.alive = {}
        self.controllerscript = None
        self.controllerlogfile = None
        self.controllerprg = None
        self.controllerok = {}
        self.refresh_every = 3
        env.password = ''
        
    def checkcontroller(self, robot):
        try:
            fabric.state.output['running']=False
            prg = self.controllerprg.split()[0]
            res=run("pidof %s"%(prg), quiet=True, warn_only=True, timeout=5)
            if res:
                self.controllerok[robot] = True
            else:
                self.controllerok[robot] = False
        except:
            print "Error checking controller for robot ", robot
            self.controllerok[robot] = False
            pass

        
    def do_runcontroller(self, robot):
        tasks.execute(self.runcontroller, \
                      robot,  hosts=["root@%s"%(self.ipof(robot))])
                
    def do_stopcontroller(self, robot):
        tasks.execute(self.stopcontroller, \
                      robot, hosts=["root@%s"%(self.ipof(robot))])
        
    def do_checkcontroller(self, robot):
        tasks.execute(self.checkcontroller, \
                      robot, hosts=["root@%s"%(self.ipof(robot))])
        
    def do_resurrect(self, robot):
        tasks.execute(self.resurrect, \
                      robot, hosts=["root@%s"%(self.ipof(robot))])
        
    
    def stopcontroller(self, robot):
        prg=self.controllerprg.split()[0]
        RUNCONTROLLER="kill -9 $(pgrep -f " + prg + ")"
        try:
            fabric.state.output['running']=False
            res=run(RUNCONTROLLER, quiet=True, warn_only=True, timeout=5)
        except:
            print "Error stopcontroller for ", robot
            pass



    def runcontroller(self, robot):
        logfile = self.controllerlogfile
        if not logfile or logfile == "":
            ctime = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            logfile = "/home/root/eduardo/logs/%s_controller.log"%(ctime)

        prg=self.controllerprg.split()[0]
        RUNCONTROLLER=("mkdir -p eduardo/logs\n"
                        "LD_LIBRARY_PATH=/home/root/ARGoS/lib:/home/root/manet/controllers "
                        "PATH=${PATH}:/home/root/manet/controllers "
                        + "nohup " + self.controllerprg + " -c " + self.controllerscript + " &> " + logfile + " & \n"
                    "ln -f -s " + logfile + " /home/root/eduardo/logs/controller.log")
        try:
            fabric.state.output['running']=True
            killcmd = "kill -9 $(pgrep -f " + prg + ")"
            res=run(killcmd, quiet=True, warn_only=True, timeout=5)
            res=run(RUNCONTROLLER, quiet=True, warn_only=False, pty=False, timeout=5)
        except:
            print "Error runcontroller for ",robot
            pass
    def resurrect(self, robot):
        CMD="resurrect"
        try:
            fabric.state.output['running']=False
            res=run(CMD, quiet=True, warn_only=True, timeout=5)
        except:
            print "Error resurrect for ", robot
            pass
    def ipof(self,robot):
        return "192.168.201.%d"%(robot+100)
    
            
    def hostalive(self, robot):
        if robot not in self.alive:
            return False
        return self.alive[robot]
    def do_checkhostalive(self, robot):
        self.alive[robot] = checkhostalive(self.ipof(robot))
        



if __name__ == "__main__":
    import sys
    robots=[int(i) for i in sys.argv[1:]]
    print "robots",robots
    n = RemoteCtrl(robots)
    n.controllerprg="footbot_jacopo_api -i footbot_jacopo_api"
    n.controllerscript="/home/root/manet/controllers/xml/ros_footbot/fb.xml"
    
    try:
        while True:
            for i in robots:
                n.do_checkhostalive(i)
                print "alive ",i,"?",n.hostalive(i)
                if n.hostalive(i):
                    n.do_checkcontroller(i)
            print "controllers ok?",n.controllerok
            for i in robots:
                if n.hostalive(i):
                    if not n.controllerok[i]:
                        n.do_runcontroller(i)
                        n.do_checkcontroller(i)
            print "controllers ok?",n.controllerok
            print "sleeping"
            time.sleep(10)
            print "stopping"
            for i in robots:
                if n.hostalive(i) and n.controllerok[i]:
                    print "stopping",i
                    n.do_stopcontroller(i)
                    
            
    except KeyboardInterrupt:
        pass


            


