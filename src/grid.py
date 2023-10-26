#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros, tf
import tf2_geometry_msgs
import math
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import time
from norbit_fls_driver.msg import Fls
__author__ = "Rayan Cali"
__author_email = "rayanc@kth.se"


def image_callback(msg: Image):
    global fade_factor, pub, sub_goal,first_image,grid,beam_min,start,bin_max,tfBuffer,listener,done_once, range_min,directions, prev_matrix, base_pos, trans_base
    t = time.time()
    cv_image = CvBridge().imgmsg_to_cv2(msg, "mono8")
    if rospy.get_param('/flip_image'):
        cv_image = cv2.flip(cv_image, 1)
    if done_once == False:
        matrix = np.zeros((grid.info.height,grid.info.width), dtype=np.int8)
    else:
        matrix = -fade_factor*np.ones((grid.info.height,grid.info.width), dtype=np.int8)
    trans = tfBuffer.lookup_transform("odom", "fls_link", rospy.Time(0))
    trans2 = tfBuffer.lookup_transform("odom", "base_link", rospy.Time(0))
    trans_base = tfBuffer.lookup_transform("base_link", "odom", rospy.Time(0)) # for use in update_matrix()
    anglelist = trans2.transform.rotation.x, trans2.transform.rotation.y, trans2.transform.rotation.z, trans2.transform.rotation.w
    yaw = tf.transformations.euler_from_quaternion(anglelist)[2]
    imagearray = np.array(cv_image)
    arr = np.where(imagearray == 255)
    step_size_range = bin_max/(msg.height)
  
    trlist = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
    rotlist = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
    mat = tf.TransformerROS().fromTranslationRotation(trlist, rotlist)

    ranges = range_min + arr[0]*step_size_range
    angle = directions[arr[1]]
   
    z = ranges*np.cos(angle)
    x = ranges*np.sin(angle)
    transform_this = np.zeros((mat.shape[0],len(arr[0])))
    transform_this[0][:] = x 
    transform_this[2][:] = z
    transform_this[3][:] = np.ones(len(arr[0]))
   
    transform_this = np.matmul(mat,transform_this)
    xpos = np.cos(yaw)
    ypos = np.sin(yaw)
    diffs = (transform_this[0,:]-trans2.transform.translation.x) * xpos + (transform_this[1,:]-trans2.transform.translation.y) * ypos
    transform_this[0,:] = diffs
    coordinates = np.delete(transform_this, [1,3], axis=0).transpose()
    coordinates[:, 1] = -(coordinates[:, 1] - trans2.transform.translation.z)
    coordinates=coordinates*1/grid.info.resolution + base_pos
    coordinates = coordinates.astype(int)
    coordinates = np.delete(coordinates, np.where(coordinates<0)[0], axis=0)
    coordinates = np.delete(coordinates, np.where((coordinates[:,1])>grid.info.height-1)[0], axis=0)
    coordinates = np.delete(coordinates, np.where((coordinates[:,0])>grid.info.width-1)[0], axis=0)
    
    
    
    matrix[coordinates[:, 1], coordinates[:, 0]] = fade_factor
    matrix[base_pos][base_pos] = fade_factor #mark where base link is on grid
    rospy.loginfo(fade_factor)
    if done_once == True:
        matrix = matrix + prev_matrix
        matrix = np.clip(matrix, 0, 100)
        prev_matrix = matrix
    else:
        done_once = True
        prev_matrix = matrix

    
    grid.data = matrix.flatten().tolist()

 
    pub.publish(grid)
    elapsed = time.time() - t 
    rospy.loginfo(elapsed)
    start = True


def fls_callback(msg: Fls):
    global directions, bin_max
    directions = np.array(msg.fls_raw.data[-msg.num_beams:])
    bin_max = ((msg.t0 + msg.num_samples)*msg.snd_velocity)/(2*msg.sample_rate)

    

def update_matrix(): #checks if matrix need to be rolled forward or backward depending on how much base_link has moved since latest image
    global matrix, prev_matrix, trans_base, tfBuffer,grid,base_pos,start
    if start:
        t = time.time()
        base_val = prev_matrix[base_pos,base_pos]
        try:
            current_base = tfBuffer.lookup_transform("odom", "base_link", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("no transform")
        diffpose = PoseStamped()
        diffpose.header.frame_id = "odom"
        diffpose.pose.position.x,diffpose.pose.position.y, diffpose.pose.position.z = current_base.transform.translation.x, current_base.transform.translation.y, current_base.transform.translation.z
        diffpose.pose.orientation.x, diffpose.pose.orientation.y,diffpose.pose.orientation.z,diffpose.pose.orientation.w = current_base.transform.rotation.x, current_base.transform.rotation.y, current_base.transform.rotation.z, current_base.transform.rotation.w
        diffpose = tf2_geometry_msgs.do_transform_pose(diffpose, trans_base)
        anglelist = diffpose.pose.orientation.x, diffpose.pose.orientation.y, diffpose.pose.orientation.z, diffpose.pose.orientation.w
        yaw = tf.transformations.euler_from_quaternion(anglelist)[2]
        if abs(yaw)<0.017: #1 degree
            dist = abs(int(diffpose.pose.position.x/grid.info.resolution))
            if diffpose.pose.position.x > grid.info.resolution:
                trans_base = tfBuffer.lookup_transform("base_link", "odom", rospy.Time(0))
                prev_matrix = np.roll(prev_matrix, -dist, axis=1)
                prev_matrix[:, -dist:] = 0
                prev_matrix[base_pos, (base_pos-dist)] = 0
                prev_matrix[base_pos,base_pos] = base_val
                rospy.loginfo("rolled fwd")
                trans_base = tfBuffer.lookup_transform("base_link", "odom", rospy.Time(0))
                
            elif diffpose.pose.position.x < -grid.info.resolution:
                trans_base = tfBuffer.lookup_transform("base_link", "odom", rospy.Time(0))
                prev_matrix = np.roll(prev_matrix, dist, axis=1)
                prev_matrix[:, 0:dist] = 0
                prev_matrix[base_pos,(base_pos+dist)] = 0
                prev_matrix[base_pos,base_pos] = base_val
                rospy.loginfo("rolled bwd")
                trans_base = tfBuffer.lookup_transform("base_link", "odom", rospy.Time(0))
        elapsed = time.time() - t
        # rospy.loginfo(elapsed)

    



rospy.init_node('grid')
done_once = False
sub_goal = rospy.Subscriber('/morphed', Image, image_callback)
fls_sub = rospy.Subscriber('/fls/data', Fls, fls_callback)
grid = OccupancyGrid()
pub = rospy.Publisher('/grid', OccupancyGrid, queue_size=1)
pub_flip = rospy.Publisher('/pic_flip', Image, queue_size=1)
first_image = True
bin_max = 200 # 100m is the max range of the sensor
beam_min = -math.pi/2 # 180 degrees is the fov of the sensor
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
st = tf2_ros.StaticTransformBroadcaster()
prev_matrix = None
base_pos = 10
range_min = 0
directions = None
trans_base = None
start = False
fade_factor = rospy.get_param('/fade_factor')



if __name__ == '__main__':
    grid.header.frame_id = "odom"
    grid.info.resolution = rospy.get_param('/grid_resolution')
    height = rospy.get_param('/grid_height')
    width = rospy.get_param('/grid_width')
    grid.info.height = int(height*1/grid.info.resolution)
    grid.info.width = int(width*1/grid.info.resolution)
    grid.info.origin = Pose(Point(0,0,0),Quaternion(0,0,0,1)) # rotated 90 degrees
    # t = TransformStamped()
    # t.header.frame_id = "base_link"
    # t.child_frame_id = "fls_link"
    # t.transform.translation.x = 0
    # t.transform.translation.y = 0
    # t.transform.translation.z = -1
    # t.transform.rotation.x = 0.923
    # t.transform.rotation.y = 0.0
    # t.transform.rotation.z = 0.383
    # t.transform.rotation.w = 0
    # st.sendTransform(t)
    base_pos = int(base_pos/grid.info.resolution)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        update_matrix() 
        rate.sleep()
    rospy.spin()
