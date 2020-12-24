#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# topics are : 
# /flappy_acc for acceleration vector
# /flappy_vel for velocity vector
# /flappy_laser_scan for sensor data            

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    # rospy.Subscriber(topic, message, callback)
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3
    # Example of publishing acceleration command on velocity velCallback
    x = 0 # replace the acceleration vector here by something depending on velocity
    y = 0
    print "Velocity: x= {}, y={}".format(msg.x, msg.y)
    pub_acc_cmd.publish(Vector3(x,y,0))

def laserScanCallback(msg):
    """ The callback is the function executed when a message is received. It takes the message as an argument, so it can display information contained in it.
        Single scan from a planar laser range-finder

        float32 angle_min = -45°        # start angle of the scan [rad]
        float32 angle_max = +45°       # end angle of the scan [rad]
        float32 angle_increment = 90/8° # angular distance between measurements [rad]

        float32 time_increment   # time between measurements [seconds] - if your scanner
                                # is moving, this will be used in interpolating position
                                # of 3d points
        float32 scan_time        # time between scans [seconds]

        float32 range_min = 0       # minimum range value [m]
        float32 range_max = 3.55        # maximum range value [m]

        float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
        float32[] intensities    # intensity data [device-specific units].  If your
                                # device does not provide intensities, please leave
                                # the array empty."""

    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range
    for k in range(len(msg.ranges)):
        print "Laser range: {}, angle: {}".format(msg.ranges[k], msg.angle_min+k*msg.angle_increment)

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
