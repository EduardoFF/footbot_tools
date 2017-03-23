#!/usr/bin/env python
"""
    Copyright (C) 2017 Eduardo Feo
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

"""
TODOS
parallelize shutdown
"""

import rospy
import roslib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import lcm
from configmsg import config_msg_t
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from remote_control import RemoteCtrl
from std_msgs.msg import ColorRGBA
import math
import time
#############################################################
#############################################################

def get_transform(tf_buffer, from_frame, to_frame):
    try:
        return tf_buffer.lookup_transform(from_frame, to_frame, rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
        return None

def pose_in_frame(tf_buffer, pose_s, frame_id):
    t = get_transform(tf_buffer, frame_id, pose_s.header.frame_id)
    if not t:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_s, t)

class FootbotControl(object):
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("footbot_control")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.message_counter =0
        self.last_pos ={}
        self.last_rgb ={}
        self.abort = False
    
        self.tgt_topic = rospy.get_param("~target", "target_pose")
        self.robots = rospy.get_param("~robots", rospy.get_param('/robots', []))
        self.lcm_url = rospy.get_param("~lcm_url","udpm://239.255.76.67:7667?ttl=1,transmit_only=true" )
        self.stop_topic = rospy.get_param("~stop", "stop")
        self.do_launch = rospy.get_param("~do_launch", 1)
        self.beaconcolor_topic = rospy.get_param("~beacon", "set_beacon")

        self.lcm = lcm.LCM(self.lcm_url )
        self.rc = RemoteCtrl(self.robots)
        self.rc.controllerprg="footbot_jacopo_api -i footbot_jacopo_api"
        self.rc.controllerscript="/home/root/manet/controllers/xml/ros_footbot/fb.xml"

        self.rate = rospy.get_param("~rate", 5)
        print "rate = ",self.rate
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        if self.do_launch:
            print "automatic launching is enabled"
            self.launch_controllers()
        for rid in self.robots:
            print "created subscribers for ",rid
            rospy.Subscriber("footbot_%d/%s"%(rid,self.tgt_topic), \
                             PoseStamped, self.targetCallback,
                             callback_args=rid)
            rospy.Subscriber("footbot_%d/%s"%(rid,self.stop_topic), \
                             Empty, self.stopCallback,
                             callback_args=rid)
            rospy.Subscriber("footbot_%d/%s"%(rid,self.beaconcolor_topic), \
                             ColorRGBA, self.beaconcolorCallback,
                             callback_args=rid)

        if self.do_launch:
            rospy.Timer(rospy.Duration(5), self.launch_controller_cb)




    def launch_controller_cb(self, event):
        self.launch_controllers();
    def launch_controllers(self):
        if self.abort:
            return
        for i in self.robots:
            self.rc.do_checkhostalive(i)
            print "alive ",i,"?",self.rc.hostalive(i)
            if self.rc.hostalive(i):
                self.rc.do_checkcontroller(i)
                if not self.rc.controllerok[i]:
                    print "controller is down on robot ",i
                    print "attempting to launch controller on robot ",i
                    print "resurrect ..."
                    self.rc.do_resurrect(i)
                    print "launch ..."
                    self.rc.do_runcontroller(i)
    def stop_controllers(self):
        self.abort = True
        for i in self.robots:
            if self.rc.hostalive(i) and self.rc.controllerok[i]:
                for j in range(3):
                    self.publishStop(i)
                    self.publishNamedBeaconColor(i, 'black')
                print "sleeping"
                time.sleep(1.0)
                print "sleep done"
                self.rc.do_stopcontroller(i)
                time.sleep(1.0)
                self.rc.do_resurrect(i)
                    
        

    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        #idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_start = 0
        #self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
            self.spinOnce()
            r.sleep()
            #print "sleep over"
            self.ticks_since_start +=1
        if self.do_launch:
            self.stop_controllers()
             

    #############################################################

    #############################################################
    def spinOnce(self):
    #############################################################
        for (rid,pos) in self.last_pos.items():
            if not pos:
                self.publishStop(rid)
            else:
                self.publishLastTarget(rid, pos)
        for (rid, color) in self.last_rgb.items():
                self.publishRGBBeaconColor(rid, color)

    def publishLastTarget(self, rid, pos):
        # testing
        msg = config_msg_t()
        msg.robotid = rid
        msg.timestamp = 0
        (x,y,o) = pos
        msg.msg = "FORCE_POS %.2f %.2f %.2f"%(x, y, o)
        self.lcm.publish("CONFIG", msg.encode())

    def publishStop(self, rid):
        # testing
        msg = config_msg_t()
        msg.robotid = rid
        msg.timestamp = 0
        msg.msg = "FORCE_STOP"
        self.lcm.publish("CONFIG", msg.encode())
    def publishNamedBeaconColor(self, rid, colorname):
        # testing
        msg = config_msg_t()
        msg.robotid = rid
        msg.timestamp = 0
        msg.msg = "FORCE_BEACON %s"% (colorname)
        self.lcm.publish("CONFIG", msg.encode())

    def publishRGBBeaconColor(self, rid, (r,g,b,a)):
        # testing
        msg = config_msg_t()
        msg.robotid = rid
        msg.timestamp = 0
        r = max(min(r,1.0),0)
        g = max(min(g,1.0),0)
        b = max(min(b,1.0),0)
        a = max(min(a,1.0),0)        
        ur = int(min(math.ceil(r*255.0),255))
        ug = int(min(math.ceil(g*255.0),255))
        ub = int(min(math.ceil(b*255.0),255))
        ua = int(min(math.ceil(a*255.0),255))
        msg.msg = "FORCE_BEACON_RGBA %d %d %d %d"% (ur, ug, ub, ua)
        self.lcm.publish("CONFIG", msg.encode())


    
    #############################################################
    def targetCallback(self,msg, rid):
    ############################################################
        self.message_counter += 1
        print "got"

        tgt_pos = pose_in_frame(self.tf_buffer, msg, 'footbot')
        if tgt_pos:
            self.last_pos[rid] = (tgt_pos.pose.position.x,\
                                  tgt_pos.pose.position.y, \
                                  0)
            (x,y,o) = self.last_pos[rid]
#            (roll, pitch, self.tgt_ori) = euler_from_quaternion(tgt_pos.pose.orientation)
            print "got (%.2f,%.2f):%.2f for robot %d"%(x,y,o, rid)
        else:
            print "can not transform"

    def stopCallback(self,msg, rid):
    ############################################################
        print "STOPPPPPPP"
        self.last_pos[rid] = None

    def beaconcolorCallback(self,msg, rid):
        ############################################################
        print "Beacon Color ",(msg.r, msg.g, msg.b, msg.a)
        self.last_rgb[rid] = (msg.r, msg.g, msg.b, msg.a)


#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    fbctrl=FootbotControl()
    fbctrl.spin()
