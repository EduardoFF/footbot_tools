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

import rospy
import roslib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import lcm
from configmsg import config_msg_t
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
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
    
        self.tgt_topic = rospy.get_param("~target", "target_pose")
        self.robots = rospy.get_param("~robots", rospy.get_param('/robots', []))
        self.lcm_url = rospy.get_param("~lcm_url","udpm://239.255.76.67:7667?ttl=1,transmit_only=true" )
        self.stop_topic = rospy.get_param("~stop", "stop")

        self.lcm = lcm.LCM(self.lcm_url )
        self.rate = rospy.get_param("~rate", 5)
        print "rate = ",self.rate
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        for rid in self.robots:
            print "created subscribers for ",rid
            rospy.Subscriber("footbot_%d/%s"%(rid,self.tgt_topic), \
                             PoseStamped, self.targetCallback,
                             callback_args=rid)
            rospy.Subscriber("footbot_%d/%s"%(rid,self.stop_topic), \
                             Empty, self.stopCallback,
                             callback_args=rid)


        
        
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
             

    #############################################################

    #############################################################
    def spinOnce(self):
    #############################################################
        for (rid,pos) in self.last_pos.items():
            if not pos:
                self.publishStop(rid)
            else:
                self.publishLastTarget(rid, pos)

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

#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    fbctrl=FootbotControl()
    fbctrl.spin()
