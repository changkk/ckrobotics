import rospy
import numpy as np
import time
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from piksi_rtk_msgs.msg import BaselineNed


# Global Vars
range_v = 0.0
gimbal_location_ned = [0, 0, 0]
pub_range = rospy.Publisher("range", Float64, queue_size=1)


# Use streamed aircraft location to calculate gimbal angles
#def HexGPSCallback(rtk_msg):
#    global range_v
#    global gimbal_location_ned

#    dist_north = rtk_msg.pose.pose.position.x - gimbal_location_ned[0]
#    dist_east = rtk_msg.pose.pose.position.y - gimbal_location_ned[1]
#    dist_down = rtk_msg.pose.pose.position.z - gimbal_location_ned[2]
#    range_v = np.sqrt(dist_east*dist_east+dist_north*dist_north+dist_down*dist_down)


def PiksiDriverGPSCallback(rtk_msg):
    global range_v
    global gimbal_location_ned
    
    dist_north = rtk_msg.n/1000.0 - gimbal_location_ned[0]
    dist_east = rtk_msg.e/1000.0 - gimbal_location_ned[1]
    dist_down = rtk_msg.d/1000.0 - gimbal_location_ned[2]
    # print rtk_msg.n
    range_v = np.sqrt(dist_north*dist_north+dist_east*dist_east+dist_down*dist_down)
    pub_range.publish(range_v)


def main():

    rospy.init_node('range_calc_node', anonymous=True)

    # rospy.Subscriber("hexacopter/gps/rtkfix", Odometry, HexGPSCallback)
    rospy.Subscriber("/pinocchio/piksi_multi/baseline_ned", BaselineNed, PiksiDriverGPSCallback)
  
    global range_v
    global gimbal_location_ned

    gimbal_location_ned = rospy.get_param('/gimbal_location_ned','[0, 0, 0]')
    # omnicam_location_ned = rospy.get_param('/omnicam_location','[0, 0, 0]')


    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
