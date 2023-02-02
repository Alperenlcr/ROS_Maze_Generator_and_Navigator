#!/usr/bin/python3
from re import L, T
from numpy.lib.function_base import select
import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import math
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from PIL import Image
from numpy.lib.function_base import average

def create_map_frame():
    image = Image.open("map.png")
    image_array = np.array(image)
    map_array = [int(average(pixel)/10) for satir in image_array for pixel in satir]
    #print(image_array)
    #print(len(map_array))
    #for kare in map_array:
    #    if kare != 128.0:
    #        print(kare)
    return map_array


# this function works only at the beginning
def create_gray_image(w=1000, h=1000):
    img = np.zeros([h,w,1],dtype=np.uint8)
    img.fill(128)
    cv.imwrite('map.png', img)

def transform_orientation(orientation_q):
    """
    Transform theta to [radians] from [quaternion orientation]
    """
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    if yaw < 0:
        yaw = 2 * np.pi + yaw  # 0->360 degrees >> 0->2pi
    return yaw


def get_odom_orientation(msgOdom):
    """"
    Get theta from Odometry msg in [radians]
    """
    orientation_q = msgOdom.pose.pose.orientation
    theta = transform_orientation(orientation_q)
    return theta
    

def get_odom_position(msgOdom):
    """
    Get (x,y) coordinates from Odometry msg in [m]
    """
    x = msgOdom.pose.pose.position.x
    y = msgOdom.pose.pose.position.y
    return (x, y)
def lidar_scan(msgScan):
    """
    Convert LaserScan msg to array
    """
    distances = np.array([])
    angles = np.array([])

    for i in range(len(msgScan.ranges)):
        # angle calculation
        ang = i * msgScan.angle_increment

        # distance calculation
        if ( msgScan.ranges[i] > msgScan.range_max ):
            dist = msgScan.range_max
        elif ( msgScan.ranges[i] < msgScan.range_min ):
            dist = msgScan.range_min
        else:
            dist = msgScan.ranges[i]

        distances = np.append(distances, dist)
        angles = np.append(angles, ang)

    # distances in [m], angles in [radians], information [0-1]
    return ( distances, angles)


def lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom):
	"""
	Lidar measurements in X-Y plane
	"""
	distances_x = np.array([])
	distances_y = np.array([])

	for (dist, ang) in zip(distances, angles):
		distances_x = np.append(distances_x, x_odom + dist * np.cos(ang + theta_odom))
		distances_y = np.append(distances_y, y_odom + dist * np.sin(ang + theta_odom))

	return (distances_x, distances_y)

rospy.init_node('mp_mapping', anonymous = False)
rate = rospy.Rate(10)
create_gray_image(1000, 800)
# Init ROS node
#rospy.init_node('map_publisher')
occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)
turtlebot_pose_br = tf.TransformBroadcaster()
# listener of transforms between the car_base_link and the world frame
#pose = tf.TransformListener()

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = 0.01
width = 1000
height = 800


# fill map_msg with the parameters from launchfile
map_msg.info.resolution = resolution
map_msg.info.width = width
map_msg.info.height = height
#map_msg.data = range(width*height)

# set map origin [meters]
map_msg.info.origin.position.x = - width // 2 * resolution
map_msg.info.origin.position.y = - height // 2 * resolution

    # stamp current ros time to the message
map_msg.header.stamp = rospy.Time.now()


# Initialize car pose relative to world
x_turtlebot = 0.0
y_turtlebot = 0.0

# square size of the car footprint [m]
footprint = 0.1

# Map update rate (defaulted to 5 Hz)
rate = 5.0

# Range data
turtlebot_range = 0.0

turtlebot_pose_br.sendTransform((0, 0, 0),
		                        tf.transformations.quaternion_from_euler(0, 0, math.pi/180*1),
		                        rospy.Time.now(),
		                        "base_link","world"
		                        )


ori = 0

while not rospy.is_shutdown():
    map = cv.imread('map.png', 0)
    # map guncellenerek degistirilmeye devam ediyor
    cv.imshow('MAP', map)
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

    # Lidar measurements
    # distances engelin robota uzakligi, angles laserin lidarin sifina gore acisi
    msgScan = rospy.wait_for_message('/scan', LaserScan)
    distances, angles = lidar_scan(msgScan)  # distances in [m], angles in [radians]

    # Odometry measurements
    msgOdom = rospy.wait_for_message('/odom', Odometry)
    x_odom, y_odom = get_odom_position(msgOdom)   # x,y in [m]
    theta_odom = get_odom_orientation(msgOdom)    # theta in [radians]

    # Lidar measurements in X-Y plane
    distances_x, distances_y = lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)

    # cmdvel twist
#    msgTwist = rospy.wait_for_message('/cmd_vel', Twist)
    
#
    #print(msgTwist.angular.z)
    #saat_yonu = False
    #if msgTwist.angular.z < 0:
    #    saat_yonu = True
    #    old_ori = ori
    #    ori = msgOdom.pose.pose.orientation.z
    #    print(theta_odom, ori)
        #print("ori:",ori)
        # robotun 0. laseri ekranin asagisina gelecek sekilde cevir
        # 0. laserin yerindeki laserin indexini orientation dan bul
        # angeles i buldugun index kadar shift et
    #    ori = int(abs(ori)*100)
        #print(distances)
        #print(type(distances))
    #    distances = list(distances)
    #    angles = list(angles)
    #    aci = 0
    #    if (saat_yonu and ori - old_ori > 0) or ((not saat_yonu) and ori - old_ori < 0): #sol
    #        distances = distances[:ori]+distances[ori:]
    #    #    angles = angles[:ori]+angles[ori:]
    #        aci += ori*(math.pi/100)
    #    else:   #sag
    #        distances = distances[:(200-ori)]+distances[(200-ori):]
    #    #    angles = angles[:(200-ori)]+angles[(200-ori):]
    #        aci += ori*(math.pi/100)
    #min()
#
    x1 = x_odom
    y1 = y_odom
    x1_pixel = int(x1*10) + 500
    y1_pixel = int(y1*10) + 400

    map = cv.circle(map, (x1_pixel, y1_pixel), radius=int(theta_odom), color=(100, 100, 100), thickness=2)
#    lidar_scan_end_points = []
#
    #print(lidar_scan_end_points)
    #for length, angle in zip(distances, angles):
    #    #length = float(length)
    #    #angle = float(angle)
    #    x2_pixel = int((x1 + length * math.cos(angle+aci))*10) + 500
    #    y2_pixel = int((y1 + length * math.sin(angle+aci))*10) + 400
    #    map = cv.line(map, (x1_pixel, y1_pixel),(x2_pixel, y2_pixel), (255, 255, 255), thickness=1)
    #    if length != 3.5:
    #        map = cv.circle(map, (x2_pixel, y2_pixel), radius=0, color=(0, 0, 0), thickness=1)
    #    length = float(length)
    #    angle = float(angle)
#
#    print("/n/n/n/n")
    for x2, y2, length in zip(distances_x, distances_y, distances):
        #print(x2, y2)
        x2 = int(x2*10) + 500
        y2 = int(y2*10) + 400
        map = cv.line(map, (x1_pixel, y1_pixel),(x2, y2), (255, 255, 255), thickness=1)
        if length != 3.5:
            map = cv.circle(map, (x2, y2), radius=0, color=(0, 0, 0), thickness=2)


    cv.imwrite('map.png', map)
    # build ros map message and publish
    map_msg.data = create_map_frame()
    occ_pub.publish(map_msg)

cv.destroyAllWindows()