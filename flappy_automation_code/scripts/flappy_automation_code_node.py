#!/usr/bin/env python
#coding=UTF-8
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# topics are : 
# /flappy_acc for acceleration vector
# /flappy_vel for velocity vector
# /flappy_laser_scan for sensor data            

EPSILON = 0.25
KP = 1 # proportional gain acceleration/distance_to_hole
KD = 0.1 # derivative gain 
y_distance_to_hole = 0

def getLowerScreenLimit(pointcloud_y,angles,intensities):
    "Detects lower screen limit"
    lower_screen_limit_y = None
    if intensities[0]: # there is an lower screen limit or a vertical wall
        if pointcloud_y[0] == pointcloud_y[1]: # lower screen limit
            print "Lower screen limit detected"
            # discard the 1th element as well
            lower_screen_limit_y = pointcloud_y[0]
        elif not intensities[1]: # the hole is at the second ray location
            print "Hole at second lowest laser ray detected"
    return lower_screen_limit_y

def getUpperScreenLimit(pointcloud_y,angles,intensities):
    "Detects upper screen limit"
    upper_screen_limit_y = None
    if intensities[-1]: # there is an upper screen limit or a rock
        if pointcloud_y[-1] == pointcloud_y[-2]: # upper screen limit
            print "Upper screen limit detected"
            # discard the -2th element as well
            upper_screen_limit_y = pointcloud_y[-1]
        elif not intensities[-2]: # the hole is at the second ray location
            print "Hole at second highest laser ray detected"
    return upper_screen_limit_y
    
def getRocksPosition(pointcloud_x, range_min, range_max):
    """ Returns the coordinates of the rock wall if there is one, returns None otherwise.
        Considers the presence of a wall if there are at least 3 points falling in the same 0.2m bin."""
        
    bin_edges = np.linspace(range_min,range_max,round((range_max-range_min)/0.3))
    distr,_ = np.histogram(pointcloud_x, bins = bin_edges)
    rocks_position = None
    possible_rocks_positions = []
    for bin_index in range(len(bin_edges)-1):  
        if distr[bin_index] >= 3:
            possible_rocks_positions.append((bin_edges[bin_index] + bin_edges[bin_index+1])/2)
    if len(possible_rocks_positions) > 0:
        rocks_position = np.min(possible_rocks_positions)        
    return rocks_position

def getHolePosition(pointcloud_x, pointcloud_y, rocks_x):
    y_distance_to_hole = None
    is_going_through = pointcloud_x - rocks_x > EPSILON
    print "{}".format(is_going_through)
    if np.sum(is_going_through) == 1: # there is one hole, with single laser going through
        y_distance_to_hole = pointcloud_y[is_going_through]    
    elif np.sum(is_going_through) > 1: # there might be one or  multiple holes
        holes_lengths, indices_holes_start = getHolesLength(is_going_through)
        y_distance_to_hole = pointcloud_y[indices_holes_start[np.argmax(holes_lengths)]] # take the biggest hole
    return y_distance_to_hole

def getHolesLength(is_going_through):
    is_going_through_diff = np.diff(np.concatenate([[False], is_going_through]).astype('int')) # concatenating a False value at first to make appear a hole start at next line if first value of is_going_through is True
    indices_holes_start = np.where(is_going_through_diff == -1)[0] # is_going_through_diff == -1 means it goes from False to True at that index in is_going_through: a hole starts at this index 
    if len(indices_holes_start) < 2: # there is only one big hole
        print "{}".format(len(indices_holes_start))
        print "{}".format(type(indices_holes_start))
        raise Warning('There are not multiple holes. Detected holes at laser index {}'.format(indices_holes_start))
        
    length_holes = []
    for index_hole_start in indices_holes_start:
        index_hole_stop = None
        next_index = index_hole_start + 1
        while not index_hole_stop: # this ends up being an infinite loop
            if next_index>=len(is_going_through_diff) or is_going_through_diff[next_index] == 1:
                index_hole_stop = next_index
                length_holes.append(index_hole_stop - index_hole_start)
            # print "Still in the while loop ... next index = {}".format(next_index)
            next_index += 1
            
    return length_holes, indices_holes_start

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
    "The velocity callback is the controller."
    # msg has the format of geometry_msgs::Vector3

    x_velocity= msg.x
    y_velocity= msg.y
        
    #print "Velocity: x= {}, y={}".format(msg.x, msg.y) # this works
    #print "Acceleration command: x={}, y={}".format(x, y)
    y_acceleration = KP*y_distance_to_hole + KD*y_velocity
    print "y_acceleration = {}".format(y_acceleration)
    pub_acc_cmd.publish(Vector3(0,y_acceleration,0))

def laserScanCallback(msg):
    """ laserScanCallback is the callback function executed when a message is received. It takes the message as an argument, so it can display and make use of information contained in it.
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
        float32[] intensities    # Is 1.0 if the laser ray has encountered an obstacle, 0.0 otherwise.
        """

    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range

    angles = msg.angle_min + np.multiply(np.arange(start=0,stop=9,dtype=int),msg.angle_increment)
    angles_deg = np.round(angles * 180/math.pi,2)
        # print "Laser range: {}, angle: {}".format(msg.ranges[k], angles[k]) # this works
    # compute points positions realtive to flappy bird:
    pointcloud_x = np.round(np.multiply(msg.ranges,np.cos(angles)),2) # precise to 1 pixel
    pointcloud_y = np.round(np.multiply(msg.ranges,np.sin(angles)),2) # precise to 1 pixel
    
    for k in range(len(msg.ranges)-1,-1,-1): # there a 9 lasers
        print "Point at {}°: x = {}, y = {}, intensity = {}".format(angles_deg[k], pointcloud_x[k], pointcloud_y[k], msg.intensities[k])
    #[np.where(walls_x == element)[0].tolist() for element in np.unique(walls_x)]

#==============================================================================
#     spring_force = np.zeros_like(msg.ranges)                      
#     for k in range(len(msg.ranges)):
#         spring_force[k] = 2*msg.ranges[k]*np.sin(angles[k])
#     total_spring_force = np.sum(spring_force)    
#     pub_acc_cmd.publish(Vector3(0,total_spring_force,0))
#==============================================================================
    if getUpperScreenLimit(pointcloud_y, angles, msg.intensities): # safe guard from upper screen limit
        upper_screen_limit_y = getUpperScreenLimit(pointcloud_y, angles, msg.intensities)
        if abs(upper_screen_limit_y) < 2*EPSILON:
            pub_acc_cmd.publish(Vector3(0,-upper_screen_limit_y,0))
    elif getLowerScreenLimit(pointcloud_y, angles, msg.intensities): # safe guard from lower screen limit
        lower_screen_limit_y = getLowerScreenLimit(pointcloud_y, angles, msg.intensities)
        if abs(lower_screen_limit_y) < 2*EPSILON:
            pub_acc_cmd.publish(Vector3(0,-lower_screen_limit_y,0))
    elif getRocksPosition(pointcloud_x, msg.range_min, msg.range_max): # rock wall
        rocks_x = getRocksPosition(pointcloud_x, msg.range_min, msg.range_max)
        print "Rock wall detected at {}m".format(round(rocks_x,2))
        if getHolePosition(pointcloud_x, pointcloud_y, rocks_x):
            y_distance_to_hole = getHolePosition(pointcloud_x, pointcloud_y, rocks_x)
            print "Hole detected at {}m above".format(y_distance_to_hole)          
        else:
            print "Hole not found."

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass