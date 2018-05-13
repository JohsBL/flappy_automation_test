import math
import rospy
from sensor_msgs.msg import LaserScan
from math import hypot
#from std.msg import Float32

class Laser:
    def __init__(self,fov,resolution,scaling):
        self.fov = fov #degrees
        self.angle_max = math.radians(fov/2.0) # radians
        self.angle_min = -math.radians(fov/2.0) # radians
        self.angle_increment = math.radians(fov/(resolution-1.0)) # radians
        self.resolution = resolution
        self.range = 355 #pixels
        self.laser_scan_publisher = rospy.Publisher("/flappy_laser_scan", LaserScan, queue_size=10)
        self.scaling = scaling

    def scan(self,startPoint,bitmap):
        pointcloud = []
        raysToCast = xrange(self.resolution)

        for i in raysToCast:
            # calc endpoint from angle and range
            angle = self.angle_min+(i*self.angle_increment)
            endPoint = (int(math.cos(angle)*self.range + startPoint[0]),
                int(-math.sin(angle)*self.range + startPoint[1]))
            #endpoint
            pointcloud.append(self._raycast(startPoint,endPoint,bitmap))
        #publish the scan
        self._publish_laser_scan(pointcloud,startPoint)
        return pointcloud

    def _raycast(self,(x0,y0),(x1,y1),bitmap):
        x0 = int(x0)
        y0 = int(y0)
        x1 = int(x1)
        y1 = int(y1)
        #calculate end point from angle and bitmap
        laserCollision = ()
        bitmapSize = bitmap.size
        #bresenhams algorithm
        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = abs(dx)
        dy = abs(dy)

        if dx > dy:
            xx, xy, yx, yy = xsign, 0, 0, ysign
        else:
            dx, dy = dy, dx
            xx, xy, yx, yy = 0, ysign, xsign, 0

        D = 2*dy - dx
        y = 0

        for x in range(dx + 1):
            pixelPos = (x0 + x*xx + y*yx, y0 + x*xy + y*yy)
            # break if out of bounds
            if not (0 <= pixelPos[0] < 432) or not (0 <= pixelPos[1] < 512):
                return laserCollision + (1,)
            # save pixelVal
            pixelVal = bitmap[pixelPos[0]][pixelPos[1]]
            #print pixelVal
            if pixelVal > 127:
                return laserCollision + (1,)
            # save val
            laserCollision = pixelPos
            if D >= 0:
                y += 1
                D -= 2*dx
            D += 2*dy
        return laserCollision + (0,)

    def _publish_laser_scan(self,pointcloud,startPoint):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser_frame'
        scan.range_min = 0.0
        scan.range_max = self.range*self.scaling
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment

        scan.ranges = []
        scan.intensities = []
        for i in range(0,len(pointcloud)):
            if pointcloud[i][2] == 1:
                dist = math.hypot(pointcloud[i][0]-startPoint[0], pointcloud[i][1]-startPoint[1])
                scan.ranges.append(self.scaling*dist)
                scan.intensities.append(pointcloud[i][2])
            else:
                scan.ranges.append(scan.range_max)
                scan.intensities.append(pointcloud[i][2])
        self.laser_scan_publisher.publish(scan)
