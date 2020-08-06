import rospy
import numpy as np
import time
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from piksi_rtk_msgs.msg import BaselineNed


# Global Vars
range_v = 0.0
gimbal_location_enu = [0, 0, 0]
pub_range = rospy.Publisher("range", Float64, queue_size=1)


# Use streamed aircraft location to calculate gimbal angles
#def HexGPSCallback(rtk_msg):
#    global range_v
#    global gimbal_location_enu

#    dist_north = rtk_msg.pose.pose.position.x - gimbal_location_enu[0]
#    dist_east = rtk_msg.pose.pose.position.y - gimbal_location_enu[1]
#    dist_down = rtk_msg.pose.pose.position.z - gimbal_location_enu[2]
#    range_v = np.sqrt(dist_east*dist_east+dist_north*dist_north+dist_down*dist_down)


def MavrosCallback(msg):
    global range_v
    global gimbal_location_enu
    
    dist_north = msg.pose.position.y/1000.0
    dist_east = msg.pose.position.x/1000.0
    dist_down = -msg.pose.position.z/1000.0
    # print rtk_msg.n
    range_v = np.sqrt(dist_north*dist_north+dist_east*dist_east+dist_down*dist_down)
    pub_range.publish(range_v)


def main():

    rospy.init_node('range_calc_node', anonymous=True)

    # rospy.Subscriber("hexacopter/gps/rtkfix", Odometry, HexGPSCallback)
    # rospy.Subscriber("/pinocchio/piksi_multi/baseline_ned", BaselineNed, PiksiDriverGPSCallback)

    rospy.Subscriber("mavros/local_position/pose", PoseStamped, MavrosCallback)

  
    global range_v
    global gimbal_location_enu

    gimbal_location_enu = rospy.get_param('/gimbal_location_enu','[0, 0, 0]')
    # omnicam_location_ned = rospy.get_param('/omnicam_location','[0, 0, 0]')


    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
