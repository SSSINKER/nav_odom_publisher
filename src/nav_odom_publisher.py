#!/usr/bin/env python
# license removed for brevity

import math
import time
from math import sin,cos,pi, trunc
from threading import current_thread

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import NavSatFix

init = True

class pos:
    def __init__(self):
        self.longitude = 0
        self.latitude = 0
        self.altitude = 0

odom = Odometry()
navsat = NavSatFix()
init_fix = NavSatFix()
pub_init_fix = rospy.Publisher('init_fix',NavSatFix, queue_size=50)

map_2_odom_GPS = tf.TransformBroadcaster()

utmpos= pos()
        
odom_quat = tf.transformations.quaternion_from_euler(0,0,0)

def callback(data):
    global init
    global init_fix

    utmpos.latitude = data.latitude
    utmpos.longitude = data.longitude
    utmpos.altitude = data.altitude

    if init == True:
        init_fix = data
        init_fix.header.frame_id = "odom_GPS"
        init = False
        rospy.set_param('/geonav_datum', [init_fix.latitude, init_fix.longitude, init_fix.altitude])
        rospy.sleep(0.01)
    
    map_2_odom_GPS.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time(),"odom_GPS","map") 
    pub_init_fix.publish(init_fix)
    time.sleep(0.1)

    


    

def mainplaying():

    # we broadcast tf that From 'map_GPS' To 'odom_GPS' below:


    pub_nav_odom_2_base_link = rospy.Publisher('nav_odom',Odometry, queue_size=50)

    rospy.Subscriber("fix", NavSatFix, callback)

    rospy.init_node('nav_odom_2_base_link_publisher',anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        odom.header.stamp = current_time
        odom.header.frame_id = "nav_odom_GPS"

        odom.pose.pose = Pose(Point(utmpos.longitude,utmpos.latitude,0), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link_GPS"
        odom.twist.twist = Twist(Vector3(0,0,0),Vector3(0,0,0))

        pub_nav_odom_2_base_link.publish(odom)

        time.sleep(0.1) # we can make the python node usage down with this code...
        rospy.Rate(10).sleep # ...Not with this code! It just works on rospy.


if __name__ == '__main__':
    try:
        mainplaying()
    except rospy.ROSInterruptException:
        pass