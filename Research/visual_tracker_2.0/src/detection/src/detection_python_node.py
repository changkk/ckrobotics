import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
import time
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from piksi_rtk_msgs.msg import BaselineNed
from geodesy import utm

# Global Vars
gimbal_pitch = 0.0
gimbal_roll = 0.0
gimbal_yaw = 0.0
range_v = 0.0
gimbal_location_ned = [0, 0, 0]
angle_msg = PoseStamped()
pub_angles = rospy.Publisher("gimbal_target_orientation", PoseStamped, queue_size=1)
pub_range = rospy.Publisher("range", Float64, queue_size=1)
bridge = CvBridge()
# omnicam_location_ned = [0, 0, 0]

# Image Resizing Parameters
# w = 640
# l = 360

# #Open video device
# cap = cv2.VideoCapture(1)
# cap.open(1)

# ret, frame1 = cap.read()
# frame1 = cv2.resize(frame1,(w, l))
# kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
# fgbg =  cv2.createBackgroundSubtractorMOG2()


#Conversion from Euler Angles to Quaternions
def euler2quat (pitch, roll, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    q = [0, 0, 0, 0]
    q[0] = cy * cr * cp + sy * sr * sp
    q[1] = cy * sr * cp - sy * cr * sp
    q[2] = cy * cr * sp + sy * sr * cp
    q[3] = sy * cr * cp - cy * sr * sp

    return q

    

# Use streamed aircraft location to calculate gimbal angles
def HexGPSCallback(rtk_msg):
    global gimbal_yaw
    global gimbal_pitch
    global gimbal_roll
    global range_v
    global gimbal_location_ned

    dist_north = rtk_msg.pose.pose.position.x - gimbal_location_ned[0]
    dist_east = rtk_msg.pose.pose.position.y - gimbal_location_ned[1]
    dist_down = rtk_msg.pose.pose.position.z - gimbal_location_ned[2]
    range_v = np.sqrt(dist_east*dist_east+dist_north*dist_north+dist_down*dist_down)
    gimbal_yaw = np.arctan2(dist_east, dist_north)
    gimbal_pitch = np.arcsin(dist_down/range_v)
 
    # Print target angle
    # str = "pyPitch: %f    Roll: %f    Yaw: %f" % (gimbal_pitch*180/3.1415, gimbal_roll*180/3.1415, gimbal_yaw*180/3.1415)
    # print str
    # print gimbal_location_ned
    # print 

def PiksiDriverGPSCallback(rtk_msg):
    global gimbal_yaw
    global gimbal_pitch
    global gimbal_roll
    global range_v
    global gimbal_location_ned
    
    dist_north = rtk_msg.n/1000 - gimbal_location_ned[0]
    dist_east = rtk_msg.e/1000 - gimbal_location_ned[1]
    dist_down = rtk_msg.d/1000 - gimbal_location_ned[2]
    # str = "D_n: %f  D_e: %f D_d: %f" % (dist_north, dist_east, dist_down)
    # print str   
    range_v = np.sqrt(dist_north*dist_north+dist_east*dist_east+dist_down*dist_down)
    gimbal_yaw = np.arctan2(dist_east, dist_north)
    gimbal_pitch = np.arcsin(dist_down/range_v)

    # Conver to quaternion
    q = euler2quat(gimbal_pitch,gimbal_roll,gimbal_yaw)
    # print q
    angle_msg.pose.orientation.x = q[0]
    angle_msg.pose.orientation.y = q[1]
    angle_msg.pose.orientation.z = q[2]
    angle_msg.pose.orientation.w = q[3]
        

    # Publish target angle for gimbal node to read and implement
    pub_angles.publish(angle_msg)
    pub_range.publish(range_v)

    # str = "g_yaw: %f  g_pit: %f" % (gimbal_yaw*180/np.pi, gimbal_pitch*180/np.pi)
    # print str   

# def omnicamCallback(msg): 
#     img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
#     cv2.imshow('Original Image',img)


# def PixhawkGPSCallback(rtk_msg):
#     global gimbal_yaw
#     global gimbal_pitch
#     global gimbal_roll
#     global range_v
#     global gimbal_location_ned

#     utm.UTMPoint utm_pt_aircraft
#     utm.UTMPoint utm_pt_base_station

#     utm_pt_aircraft = utm.fromLatLong(rtk_msg.latitude,rtk_msg.longitude,rtk_msg.altitude)
#     utm_pt_base_station = utm.fromLatLong(gimbal_location_wgs[0], gimbal_location_wgs[1], gimbal_location_wgs[2])




#     #mavros/global_position/local message in UTM CS, most likely ENU!
#     dist_north = rtk_msg.pose.pose.position.y - gimbal_location_ned[0]
#     dist_east = rtk_msg.pose.pose.position.x - gimbal_location_ned[1]
#     dist_down = -rtk_msg.pose.pose.position.z - gimbal_location_ned[2]
#     range_v = np.sqrt(dist_east*dist_east+dist_north*dist_north+dist_down*dist_down)
#     gimbal_yaw = np.arctan2(dist_east, dist_north)
#     gimbal_pitch = np.arcsin(dist_down/range_v)
 
#     # Print target angle
#     # str = "pyPitch: %f    Roll: %f    Yaw: %f" % (gimbal_pitch*180/3.1415, gimbal_roll*180/3.1415, gimbal_yaw*180/3.1415)
#     # print str
#     # print gimbal_location_ned
#     # print 




# Set Mavlink streaming rates upon startup
# def set_rates(a, b, c)
# {
# 	# mavros_msgs::StreamRate srv;
#     rospy.wait_for_service('add_two_ints')
	
#     set_stream_rates = rospy.ServiceProxy('')

#     srv.request.stream_id = a;
# 	srv.request.message_rate = b;
# 	srv.request.on_off = c;

# 	if (client.call(srv))
# 	{
# 	    ROS_INFO("Stream Rates set ");
# 	}
# 	else
# 	{
# 	    ROS_ERROR("Failed to set stream rates");
# 	}
# }	


    

def main():
    # pub_angles = rospy.Publisher("gimbal_target_orientation", PoseStamped, queue_size=1)
    # pub_range = rospy.Publisher("range", Float64, queue_size=1)

    rospy.init_node('detection_node', anonymous=True)

    rospy.Subscriber("hexacopter/gps/rtkfix", Odometry, HexGPSCallback)
    rospy.Subscriber("piksi/baseline_ned", BaselineNed, PiksiDriverGPSCallback)
    # rospy.Subscriber("aircraft/mavros/global_position/local", PoseWithCovarianceStamped, PixhawkGPSCallback)
    # rospy.Subscriber("aircraft/mavros/global_position/global", NavSatFix, PixhawkGPSCallback)

    # rospy.Subscriber("camera/image_raw/compressed", CompressedImage, omnicamCallback)


    global gimbal_pitch
    global gimbal_roll
    global gimbal_yaw    
    global range_v
    global gimbal_location_ned

    gimbal_location_ned = rospy.get_param('/gimbal_location_ned','[1, 0, 0]')
    # omnicam_location_ned = rospy.get_param('/omnicam_location','[0, 0, 0]')

    rate = rospy.Rate(30)

    # angle_msg = PoseStamped()

    

    
    while not rospy.is_shutdown():

        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)

        # pitch = (3.1415/4)*np.sin(0.1/6*count)
        # roll = 0
        # yaw = (3.1415/4)*np.cos(0.1/6*count)+(3.1415/2)
        # count += 1

        # ret, frame2 = cap.read()
        # frame2 = cv2.resize(frame2,(w, l))

        # fgmask = fgbg.apply(frame2)
        # fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        # cv2.imshow('Original Image',frame2)
        # cv2.imshow('Background Mask',fgmask)
        # k = cv2.waitKey(30) & 0xff

        # # Conver to quaternion
        # q = euler2quat(gimbal_pitch,gimbal_roll,gimbal_yaw)
        # # print q
        # angle_msg.pose.orientation.x = q[0]
        # angle_msg.pose.orientation.y = q[1]
        # angle_msg.pose.orientation.z = q[2]
        # angle_msg.pose.orientation.w = q[3]
        

        # # Publish target angle for gimbal node to read and implement
        # pub_angles.publish(angle_msg)
        # pub_range.publish(range_v)
        rate.sleep()
        # rospy.spin()

    # cap.release()
    # cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
