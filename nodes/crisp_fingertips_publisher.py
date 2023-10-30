#!/usr/bin/env python

from __future__ import division, absolute_import, print_function

import sys
import logging
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Point, WrenchStamped
from crisp_fingertips.msg import xServerMsg
import numpy as np


class CrispFingertipNode(object):
    """
    ROS interface for Crisp fingertips sensors
    """
    first_received_th=0;first_received_in=0;first_received_mid=0;first_received_ring=0

    offset_z0_th=1912.0;offset_z1_th=2214.0;offset_z2_th=2158.0;offset_z3_th=2011.0
    offset_z0_in=1879.0;offset_z1_in=1873.0;offset_z2_in=2163.0;offset_z3_in=2058.0
    offset_z0_mid=1728.0;offset_z1_mid=1859.0;offset_z2_mid=1939.0;offset_z3_mid=1926.0
    offset_z0_ring=1767.0;offset_z1_ring=1924.0;offset_z2_ring=1625.0;offset_z3_ring=1829.0


    def __init__(self):
        # """
        # Initialize OptoforceDriver object
        # """
        self.arduino_sub = rospy.Subscriber("/xServTopic", xServerMsg, self.__OnMuxReceived)

        self.crisp_th_pub=rospy.Publisher("/Crisp_TH_2HGlove", WrenchStamped, queue_size=5)
        self.crisp_in_pub=rospy.Publisher("/Crisp_IN_2HGlove", WrenchStamped, queue_size=5)
        self.crisp_mid_pub=rospy.Publisher("/Crisp_MID_2HGlove", WrenchStamped, queue_size=5)
        self.crisp_ring_pub=rospy.Publisher("/Crisp_RING_2HGlove", WrenchStamped, queue_size=5)
        
        self.crisp_th4_pub=rospy.Publisher("/Crisp_TH4", WrenchStamped, queue_size=5)
        self.crisp_in4_pub=rospy.Publisher("/Crisp_IN4", WrenchStamped, queue_size=5)
        self.crisp_mid4_pub=rospy.Publisher("/Crisp_MID4", WrenchStamped, queue_size=5)
        self.crisp_ring4_pub=rospy.Publisher("/Crisp_RING4", WrenchStamped, queue_size=5)
        
        # rospy.logwarn('Crisp Fingertips MUST be run when the hand is open !')

        rospy.loginfo('Crisp Fingertips: Sensing... !')

        rospy.spin()

    def __OnMuxReceived(self, force_sensed):
        # rospy.logfatal("force sensed: %f", force_sensed.points[3].point.x) #0-3

##TH_ID=1 MUX3 -- RING_ID=2 MUX4 -- MID_ID=3 MUX5 -- IN_ID=4 MUX6

#  THUMB
        if force_sensed.sensorid == 1:
            if self.first_received_th==0:
                if force_sensed.points[0].point.z>-10000:
                    self.offset_z0_th=force_sensed.points[0].point.z
                    self.offset_z1_th=force_sensed.points[1].point.z
                    self.offset_z2_th=force_sensed.points[2].point.z
                    self.offset_z3_th=force_sensed.points[3].point.z
                    print("offset_z1,offset_z2,offset_z3,offset_z4", self.offset_z0_th,self.offset_z1_th,self.offset_z2_th,self.offset_z3_th)
                    self.first_received_th=1
                
        #normalized force
            n_m_x1=force_sensed.points[0].point.x - (-755);      n_m_x2=force_sensed.points[1].point.x - (-872)
            n_m_x3=force_sensed.points[2].point.x - (38);        n_m_x4=force_sensed.points[3].point.x - (57)

            n_m_y1=force_sensed.points[0].point.y - (-1096);      n_m_y2=force_sensed.points[1].point.y - (391)
            n_m_y3=force_sensed.points[2].point.y - (164);        n_m_y4=force_sensed.points[3].point.y - (-1258)

            n_m_z1=force_sensed.points[0].point.z - (self.offset_z0_th);       n_m_z2=force_sensed.points[1].point.z - (self.offset_z1_th)
            n_m_z3=force_sensed.points[2].point.z - (self.offset_z2_th);       n_m_z4=force_sensed.points[3].point.z - (self.offset_z3_th)

            th_force = WrenchStamped()
            th_force4 = WrenchStamped()


            free_offset_x=0.0;free_offset_y=-0.0;free_offset_z=0.0; #offset found with no contact
            th_force.wrench.force.x=free_offset_x+ np.max([n_m_x1,n_m_x2,n_m_x3,n_m_x4])#+np.std([n_m_x1,n_m_x2,n_m_x3,n_m_x4])
            th_force.wrench.force.y=free_offset_y+ np.max([n_m_y1,n_m_y2,n_m_y3,n_m_y4])#+np.std([n_m_y1,n_m_y2,n_m_y3,n_m_y4])
            th_force.wrench.force.z=free_offset_z+ np.max([n_m_z1,n_m_z2,n_m_z3,n_m_z4])#+np.std([n_m_z1,n_m_z2,n_m_z3,n_m_z4])
            # free_offset_x=-90.0;free_offset_y=0.0;free_offset_z=0.0; #offset found with no contact
            # th_force.wrench.force.x=free_offset_x+max([n_m_x1,n_m_x2,n_m_x3,n_m_x4])#, key=abs) 
            # th_force.wrench.force.y=free_offset_y+max([n_m_y1,n_m_y2,n_m_y3,n_m_y4])#, key=abs)    
            # th_force.wrench.force.z=free_offset_z+max([n_m_z1,n_m_z2,n_m_z3,n_m_z4])#, key=abs)
            th_force4.wrench.force.x=n_m_z1
            th_force4.wrench.force.y=n_m_z2
            th_force4.wrench.force.z=n_m_z3
            th_force4.wrench.torque.x=n_m_z4

            th_force.header.frame_id="/Crisp_TH_2HGlove"
            self.crisp_th4_pub.publish(th_force4)     
            self.crisp_th_pub.publish(th_force)
     
# #  RING
        elif force_sensed.sensorid == 2:
            if self.first_received_ring==0:
                if force_sensed.points[0].point.z>-10000:
                    self.offset_z0_ring=force_sensed.points[0].point.z
                    self.offset_z1_ring=force_sensed.points[1].point.z
                    self.offset_z2_ring=force_sensed.points[2].point.z
                    self.offset_z3_ring=force_sensed.points[3].point.z
                    # print("offset_z1,offset_z2,offset_z3,offset_z4", self.offset_z0_ring,self.offset_z1_ring,self.offset_z2_ring,self.offset_z3_ring)
                    self.first_received_ring=1

            n_m_x1=force_sensed.points[0].point.x - (-639);       n_m_x2=force_sensed.points[1].point.x - (-500)
            n_m_x3=force_sensed.points[2].point.x - (419);        n_m_x4=force_sensed.points[3].point.x - (345)

            n_m_y1=force_sensed.points[0].point.y - (-1040);      n_m_y2=force_sensed.points[1].point.y - (655)
            n_m_y3=force_sensed.points[2].point.y - (718);        n_m_y4=force_sensed.points[3].point.y - (-1077)

            # n_m_z1=force_sensed.points[0].point.z - (2629);       n_m_z2=force_sensed.points[1].point.z - (2636)
            # n_m_z3=force_sensed.points[2].point.z - (2345);       n_m_z4=force_sensed.points[3].point.z - (2660)
            #soft
            n_m_z1=force_sensed.points[0].point.z - (self.offset_z0_ring);       n_m_z2=force_sensed.points[1].point.z - (self.offset_z1_ring)
            n_m_z3=force_sensed.points[2].point.z - (self.offset_z2_ring);       n_m_z4=force_sensed.points[3].point.z - (self.offset_z3_ring)

            ring_force = WrenchStamped()
            ring_force4 = WrenchStamped()

            free_offset_x=0.0;free_offset_y=0.0;free_offset_z=0.0; #offset found with no contact
            ring_force.wrench.force.x=free_offset_x+np.max([n_m_x1,n_m_x2,n_m_x3,n_m_x4])#+np.std([n_m_x1,n_m_x2,n_m_x3,n_m_x4])
            ring_force.wrench.force.y=free_offset_y+np.max([n_m_y1,n_m_y2,n_m_y3,n_m_y4])#+np.std([n_m_y1,n_m_y2,n_m_y3,n_m_y4])   
            ring_force.wrench.force.z=free_offset_z+np.max([n_m_z1,n_m_z2,n_m_z3,n_m_z4])#+np.std([n_m_z1,n_m_z2,n_m_z3,n_m_z4])
            ring_force4.wrench.force.x=n_m_z1
            ring_force4.wrench.force.y=n_m_z2
            ring_force4.wrench.force.z=n_m_z3          
            ring_force4.wrench.torque.x=n_m_z4

            ring_force.header.frame_id="/Crisp_RING_2HGlove"
            self.crisp_ring4_pub.publish(ring_force4)     
            self.crisp_ring_pub.publish(ring_force)
# #  MID
        elif force_sensed.sensorid == 3:
            if self.first_received_mid==0:
                if force_sensed.points[0].point.z>-10000:
                    self.offset_z0_mid=force_sensed.points[0].point.z
                    self.offset_z1_mid=force_sensed.points[1].point.z
                    self.offset_z2_mid=force_sensed.points[2].point.z
                    self.offset_z3_mid=force_sensed.points[3].point.z
                    self.first_received_mid=1

            n_m_x1=force_sensed.points[0].point.x - (-975);      n_m_x2=force_sensed.points[1].point.x - (-810)
            n_m_x3=force_sensed.points[2].point.x - (364);       n_m_x4=force_sensed.points[3].point.x - (331)

            n_m_y1=force_sensed.points[0].point.y - (-1230);     n_m_y2=force_sensed.points[1].point.y - (776)
            n_m_y3=force_sensed.points[2].point.y - (826);       n_m_y4=force_sensed.points[3].point.y - (-1275)

            n_m_z1=force_sensed.points[0].point.z - (self.offset_z0_mid);      n_m_z2=force_sensed.points[1].point.z - (self.offset_z1_mid)
            n_m_z3=force_sensed.points[2].point.z - (self.offset_z2_mid);      n_m_z4=force_sensed.points[3].point.z - (self.offset_z3_mid)

            mid_force = WrenchStamped()
            mid4_force = WrenchStamped()

            free_offset_x=0.0;free_offset_y=00.0;free_offset_z=0.0; #offset found with no contact
            # print("std_dev_mid /n",np.std([n_m_z1,n_m_z2,n_m_z3,n_m_z4]))
            # print("variance \n",np.var([n_m_z1,n_m_z2,n_m_z3,n_m_z4]))
            mid_force.wrench.force.x=free_offset_x+np.max([n_m_x1,n_m_x2,n_m_x3,n_m_x4])#+np.std([n_m_x1,n_m_x2,n_m_x3,n_m_x4])
            mid_force.wrench.force.y=free_offset_y+np.max([n_m_y1,n_m_y2,n_m_y3,n_m_y4])#+np.std([n_m_y1,n_m_y2,n_m_y3,n_m_y4])   
            mid_force.wrench.force.z=free_offset_z+np.max([n_m_z1,n_m_z2,n_m_z3,n_m_z4])#+np.std([n_m_z1,n_m_z2,n_m_z3,n_m_z4])
            mid4_force.wrench.force.x=n_m_z1
            mid4_force.wrench.force.y=n_m_z2
            mid4_force.wrench.force.z=n_m_z3
            mid4_force.wrench.torque.x=n_m_z4

            mid_force.header.frame_id="/Crisp_MID_2HGlove"

            self.crisp_mid4_pub.publish(mid4_force)     
            self.crisp_mid_pub.publish(mid_force)
# #  INDEX
        elif force_sensed.sensorid == 4:
            if self.first_received_in==0:
                if force_sensed.points[0].point.z>-10000:
                    self.offset_z0_in=force_sensed.points[0].point.z
                    self.offset_z1_in=force_sensed.points[1].point.z
                    self.offset_z2_in=force_sensed.points[2].point.z
                    self.offset_z3_in=force_sensed.points[3].point.z
                    self.first_received_in=1            
            n_m_x1=force_sensed.points[0].point.x - (-394);      n_m_x2=force_sensed.points[1].point.x - (-286)
            n_m_x3=force_sensed.points[2].point.x - (994);       n_m_x4=force_sensed.points[3].point.x - (682)

            n_m_y1=force_sensed.points[0].point.y - (-778);     n_m_y2=force_sensed.points[1].point.y - (1023)
            n_m_y3=force_sensed.points[2].point.y - (850);      n_m_y4=force_sensed.points[3].point.y - (-940)

            # n_m_z1=force_sensed.points[0].point.z - (2879);      n_m_z2=force_sensed.points[1].point.z - (2541)
            # n_m_z3=force_sensed.points[2].point.z - (2423);      n_m_z4=force_sensed.points[3].point.z - (2555)

            #soft
            n_m_z1=force_sensed.points[0].point.z - (self.offset_z0_in);      n_m_z2=force_sensed.points[1].point.z - (self.offset_z1_in)
            n_m_z3=force_sensed.points[2].point.z - (self.offset_z2_in);      n_m_z4=force_sensed.points[3].point.z - (self.offset_z3_in)

            in_force = WrenchStamped()
            in_force4 = WrenchStamped()

            free_offset_x=0.0;free_offset_y=0.0;free_offset_z=0.0; #offset found with no contact
            # print("avg index \n",(n_m_z1+n_m_z2+n_m_z3+n_m_z4)/4)

            in_force.wrench.force.x=free_offset_x+np.max([n_m_x1,n_m_x2,n_m_x3,n_m_x4])#+np.std([n_m_x1,n_m_x2,n_m_x3,n_m_x4])
            in_force.wrench.force.y=free_offset_y+np.max([n_m_y1,n_m_y2,n_m_y3,n_m_y4])#+np.std([n_m_y1,n_m_y2,n_m_y3,n_m_y4])
            in_force.wrench.force.z=free_offset_z+np.max([n_m_z1,n_m_z2,n_m_z3,n_m_z4])#+np.std([n_m_z1,n_m_z2,n_m_z3,n_m_z4]) 
            in_force4.wrench.force.x=n_m_z1
            in_force4.wrench.force.y=n_m_z2
            in_force4.wrench.force.z=n_m_z3
            in_force4.wrench.torque.x=n_m_z4

            in_force.header.frame_id="/Crisp_IN_2HGlove"
            self.crisp_in4_pub.publish(in_force4)     
            self.crisp_in_pub.publish(in_force)




if __name__ == '__main__':
    rospy.init_node("crisp_fingertips_pub")
    try:
        node = CrispFingertipNode()
    except Exception as e:
        rospy.logfatal("Caught exception: " + str(e))
    else:
        logging.getLogger('Crisp Sensor Pub').setLevel(logging.DEBUG)

        # node.config()
        # node.run()