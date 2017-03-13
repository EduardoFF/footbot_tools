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
import lcm
from configmsg import config_msg_t
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from footbot_control import FootbotControl

class MutiFootbotControl(FootbotControl):
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("footbot_control")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.tgt_x = None
        self.tgt_y = None
        self.tgt_ori = None
        self.message_counter =0
    
        self.tgt_topic = rospy.get_param("~target", "target")
        self.robot_id = rospy.get_param("~robot_id", 1)
        self.lcm_url = rospy.get_param("~lcm_url","udpm://239.255.76.67:7667?ttl=1,transmit_only=true" )
        self.stop_topic = rospy.get_param("~stop", "stop")
        
        rospy.Subscriber(self.tgt_topic, \
                         PoseStamped, self.targetCallback)
        self.lcm = lcm.LCM(self.lcm_url )
        self.rate = rospy.get_param("~rate", 5)
        print "rate = ",self.rate
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        
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
        self.publishLastTarget()

    def publishLastTarget(self):
        if self.tgt_x  == None or \
           self.tgt_y == None or \
           self.tgt_ori == None:
            return
        # testing
        msg = config_msg_t()
        msg.robotid = self.robot_id
        msg.timestamp = 0        
        msg.msg = "FORCE_POS %.2f %.2f %.2f"%(self.tgt_x, self.tgt_y, self.tgt_ori)
        self.lcm.publish("CONFIG", msg.encode())


    
    #############################################################
    def targetCallback(self,msg):
    ############################################################
        self.message_counter += 1

        tgt_pos = pose_in_frame(self.tf_buffer, msg, 'footbot')
        if tgt_pos:
            self.tgt_x = tgt_pos.pose.position.x
            self.tgt_y = tgt_pos.pose.position.y
            self.tgt_ori = 0
#            (roll, pitch, self.tgt_ori) = euler_from_quaternion(tgt_pos.pose.orientation)
            print "got (%.2f,%.2f):%.2f"%(self.tgt_x, self.tgt_y, self.tgt_ori)
        else:
            print "can not transform"
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    fbctrl=FootbotControl()
    fbctrl.spin()
