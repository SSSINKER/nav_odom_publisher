#!/usr/bin/env python
# license removed for brevity

import math
import time
from math import sin,cos,pi, trunc
from threading import current_thread

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TwistStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

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

lin_vel = Vector3(0, 0, 0)
ang_vel = Vector3(0, 0, 0)

base_link_GPS_2_base_link_tf = tf.TransformBroadcaster()

utmpos = pos()
utmori = Quaternion()
        
def callback_imu_data(data):
    global utmori, ang_vel
    # subscribed data(type: geometry_msgs.Quaternion) --> euler(type: tuple)
    euler = tf.transformations.euler_from_quaternion((data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))

    ang_vel = data.angular_velocity

    # euler --> quaternion (type: numpy.ndarray)
    # We can't change tuple elements, so we just change the rotation axis direction on this line below.
    quaternion = tf.transformations.quaternion_from_euler(euler[0],-euler[1],-euler[2])

    # quaternion --> utmori
    utmori.x = quaternion[0]
    utmori.y = quaternion[1]
    utmori.z = quaternion[2]
    utmori.w = quaternion[3]


def callback_gps_vel(data):
    global lin_vel

    lin_vel.x = data.twist.linear.x if data.twist.linear.x != "nan" else 0
    lin_vel.y = data.twist.linear.y if data.twist.linear.y != "nan" else 0
    lin_vel.z = 0


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

    pub_init_fix.publish(init_fix)


def mainplaying():

    # we broadcast tf that From 'map_GPS' To 'odom_GPS' below:
    nav_odom_pub = rospy.Publisher('nav_odom',Odometry, queue_size=50)

    rospy.Subscriber("fix", NavSatFix, callback)
    rospy.Subscriber("vel", TwistStamped, callback_gps_vel)
    rospy.Subscriber("imu_data", Imu, callback_imu_data)
    rospy.init_node('nav_odom_publisher',anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        global utmori, lin_vel, ang_vel

        current_time = rospy.Time.now()
        odom.header.stamp = current_time
        odom.header.frame_id = "nav_odom_GPS"
        odom.pose.pose.orientation = utmori
        odom.pose.pose.position.x = utmpos.longitude
        odom.pose.pose.position.y = utmpos.latitude
        odom.pose.pose.position.z = 0

        odom.twist.twist.linear = lin_vel
        odom.twist.twist.angular = ang_vel

        odom.child_frame_id = "base_link_GPS"

        nav_odom_pub.publish(odom)

        base_link_GPS_2_base_link_tf.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "base_link",
            "base_link_GPS")
        
        rate.sleep()


if __name__ == '__main__':
    try:
        mainplaying()
    except rospy.ROSInterruptException:
        pass