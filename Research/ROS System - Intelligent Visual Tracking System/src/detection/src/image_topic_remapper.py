import rospy
import numpy as np
import time
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image


gige_pub = rospy.Publisher("gige_image_5hz", Image, queue_size=1)
omni_pub = rospy.Publisher("omni_image_5hz", Image, queue_size=1)
undistort_pub = rospy.Publisher("undistort_fisheye_5hz", Image, queue_size=1)


gige_img = Image()
omni_img = Image()
undistort_img = Image()


def GigeCallback(msg):
    global gige_img
    gige_img = msg

def OmniCallback(msg):
    global omni_img
    omni_img = msg

def undistortCallback(msg):
    global undistort_img
    undistort_img = msg




def main():

    rospy.init_node('image_topic_remapper_node', anonymous=True)

    rospy.Subscriber("camera/image_raw", Image, GigeCallback)
    rospy.Subscriber("omnicam/image_raw", Image, OmniCallback)
    rospy.Subscriber("undistort_fisheye", Image, undistortCallback)


    r = rospy.Rate(5)

    global gige_img
    global omni_img
    global undistort_img

    while not rospy.is_shutdown():
        gige_pub.publish(gige_img)
        omni_pub.publish(omni_img)
        undistort_pub.publish(undistort_img)
        r.sleep()


    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
