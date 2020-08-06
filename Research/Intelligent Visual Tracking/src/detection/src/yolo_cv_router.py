# #!/usr/bin/env python
# from __future__ import print_function

# import roslib
# # roslib.load_manifest('my_package')
# import sys
# import rospy
# import cv2
# import numpy as np
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge, CvBridgeError
# import darknet

# class image_converter:

#   def __init__(self):
#     self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

#     self.bridge = CvBridge()
#     self.image_sub = rospy.Subscriber("omnicam/image_raw/compressed",CompressedImage,self.callback)

#   def callback(self,msg):
    
#     nparr = np.fromstring(msg.data, np.uint8)
#     img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

#     try:
#       cv_image = self.bridge.imgmsg_to_cv2(img)
#     except CvBridgeError as e:
#       print(e)

#     (rows,cols,channels) = cv_image.shape
#     if cols > 60 and rows > 60 :
#       cv2.circle(cv_image, (50,50), 10, 255)

#     cv2.imshow("Image window", cv_image)
#     cv2.waitKey(3)

#     try:
#       self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#     except CvBridgeError as e:
#       print(e)

# def main():
#   ic = image_converter()
#   rospy.init_node('detection_node', anonymous=True)
# #   rospy.Subscriber("omnicam/image_raw/compressed", CompressedImage, omnicamCallback)
  
# #   net = darknet.load_net(b"/home/usl/darknet/cfg/yolov2.cfg", b"/home/usl/darknet/yolov2.weights", 0)
# #   meta = darknet.load_meta(b"/home/usl/darknet/cfg/coco.data")
# #   cv2.namedWindow("img", cv2.WINDOW_NORMAL)

# #   while not rospy.is_shutdown():
# #     r = detect_np(net, meta, img)
# #     # print(r)
# #     for i in r:
# #         x, y, w, h = i[2][0], i[2][1], i[2][2], i[2][3]
# #         xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
# #         pt1 = (xmin, ymin)
# #         pt2 = (xmax, ymax)
# #         cv2.rectangle(img, pt1, pt2, (0, 255, 0), 2)
# #         cv2.putText(img, i[0].decode() + " [" + str(round(i[1] * 100, 2)) + "]", (pt1[0], pt1[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 0], 4)
# #         cv2.imshow("img", img)
# #         k = cv2.waitKey(1)
# #         if k == 27:
# #             cv2.destroyAllWindows()
# #             exit()


#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main(sys.argv)


#!/usr/bin/env python

"""
Created on 17/11/16
@author: Sam Pfeiffer <sammypfeiffer@gmail.com>
Skeleton of a OpenCV node to subscribe to
either Image or CompressedImage topic and do
some work on the image and publish it
in a Image and CompressedImage topics
(Python has no publisher for doing both
as far as I know).
"""

import rospy
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import darknet

x_old = 0
no_meas_counter = 5
w = 0
h = 0

mp = np.array((2,1), np.float32) # measurement
est = np.zeros((2,1), np.float32) # tracked / prediction
cor = np.zeros((2,1), np.float32) 

kalman = cv2.KalmanFilter(4,2)
kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 0.5
#kalman.measurementNoiseCov = np.array([[1,0],[0,1]],np.float32) * 0.00003

class OpenCVSkeleton(object):
    def __init__(self):
        # Set CvBridge
        self.bridge = CvBridge()
        # Store last image to process here
        self.last_img = None
        # Also a handy flag if the image is new
        self.is_new_img = False
        # Allow to subscribe to both Image and CompressedImage
        img_in_topicname = rospy.resolve_name('camera/image_raw/compressed')
        # img_in_topicname = rospy.resolve_name('omnicam/image_raw/compressed')
        
        type_name, img_in_topicname, _ = get_topic_type(img_in_topicname)
        if type_name == 'sensor_msgs/Image':
            self.sub = rospy.Subscriber(img_in_topicname,
                                        Image,
                                        self.img_cb,
                                        queue_size=1)
        elif type_name == 'sensor_msgs/CompressedImage':
            self.sub = rospy.Subscriber(img_in_topicname,
                                        CompressedImage,
                                        self.img_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to: " + self.sub.resolved_name +
                      " of type: " + str(self.sub.type))

        img_out_topicname = rospy.resolve_name('/img_out')
        self.pub = rospy.Publisher(img_out_topicname,
                                   Image,
                                   queue_size=1)
        self.pub_compressed = rospy.Publisher(img_out_topicname +
                                              '/compressed',
                                              CompressedImage,
                                              queue_size=1)
        self.pub_yolo_detection = rospy.Publisher('yolo_detection_box',
                                              PolygonStamped,
                                              queue_size=1)

    def img_to_cv2(self, image_msg):
        """
        Convert the image message into a cv2 image (numpy.ndarray)
        to be able to do OpenCV operations in it.
        :param Image or CompressedImage image_msg: the message to transform
        """
        # rospy.loginfo("image is of type: " + str(type(image_msg)))
        type_as_str = str(type(image_msg))
        if type_as_str.find('sensor_msgs.msg._CompressedImage.CompressedImage') >= 0:
            # Image to numpy array
            np_arr = np.fromstring(image_msg.data, np.uint8)
            # Decode to cv2 image and store
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif type_as_str.find('sensor_msgs.msg._Image.Image') >= 0:
            # Use CvBridge to transform
            try:
                return self.bridge.imgmsg_to_cv2(image_msg,
                                                 image_msg.encoding)  # "bgr8"
            except CvBridgeError as e:
                rospy.logerr("Error when converting image: " + str(e))
                return None
        else:
            rospy.logerr("We don't know how to transform image of type " +
                         str(type(image_msg)) + " to cv2 format.")
            return None

    def img_cb(self, image):
        """
        Callback for the Image or Compressed image subscriber, storing
        this last image and setting a flag that the image is new.
        :param Image or CompressedImage image: the data from the topic
        """
        self.last_img = image
        self.is_new_img = True

    def pub_images(self, cv2_img, image_format="passthrough"):
        """
        Publish onto the Image and CompressedImage topics if there is any
        subscriber.
        :param numpy.ndarray cv2_img: image in cv2 format to publish
        :param str image_format: the image format in which the image should
            be transformed into, list available at:
            http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
        """
        if self.pub.get_num_connections() > 0:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(cv2_img, image_format)
                self.pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr("Error on converting image for publishing: " +
                             str(e) + " (Is your image_format correct?)")

        if self.pub_compressed.get_num_connections() > 0:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv2_img)[1]).tostring()
            self.pub_compressed.publish(msg)

    def do_stuff(self, net, meta):
        """
        Method to do stuff with the last image received.
        First we transform the image message to a cv2 image (numpy.ndarray).
        Then we do OpenCV stuff with it.
        And we publish the new image.
        """
        cv2_img = self.img_to_cv2(self.last_img)
        # Now we can use cv2 functions as the image is <type 'numpy.ndarray'>
        # rospy.loginfo("cv2_img: " + str(type(cv2_img)))
        # Your OpenCV stuff
        # cv2_img = cv2.resize(cv2_img, (0,0), fx=0.25, fy=0.25) 

        (rows,cols,channels) = cv2_img.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv2_img, (50,50), 10, 255)
       
        global x_old
        global no_meas_counter
        global est
        global cor
        global w
        global h
        

        r = darknet.detect(net, meta, cv2_img)
        # print(r)

        if not r:
            no_meas_counter += 1

        for i in r:
            if i[0].decode() == "person":
                x, y, w, h = i[2][0], i[2][1], i[2][2], i[2][3]
                xmin, ymin, xmax, ymax = darknet.convertBack(float(x), float(y), float(w), float(h))
                pt1 = (xmin, ymin)
                pt2 = (xmax, ymax)
                cv2.rectangle(cv2_img, pt1, pt2, (0, 255, 0), 2)
                cv2.putText(cv2_img, i[0].decode() + " [" + str(round(i[1] * 100, 2)) + "]", (pt1[0], pt1[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 0], 4)
                
                global mp
                mp = np.array([[np.float32(x)],[np.float32(y)]])
                cor = kalman.correct(mp)
                no_meas_counter = 0
		

            else:
                no_meas_counter += 1
            
                # x_old = x

            # cv2.imshow("cv2_img", cv2_img)
            # k = cv2.waitKey(1)
            # if k == 27:
            #     cv2.destroyAllWindows()
            #     exit()

        if no_meas_counter < 30:
            est = kalman.predict()
            msg = PolygonStamped()
            msg.header.stamp = rospy.Time.now()
            # msg.polygon.points = [Point32(x=x, y=y), Point32(x=cols, y=rows), Point32(x=w, y=h)]
            msg.polygon.points = [Point32(x=est[0], y=est[1]), Point32(x=cols, y=rows), Point32(x=w, y=h)]        
            self.pub_yolo_detection.publish(msg)

        # cv2.imshow("Image window", cv2_img)
        # cv2.waitKey(3)

        self.pub_images(cv2_img)
        self.is_new_img = False

    # def kf_call(r):



    def run(self):
        """
        Method to do stuff at a certain rate.
        """
        r = rospy.Rate(30)

        net = darknet.load_net(b"/home/nvidia/darknet/cfg/yolov3-tiny.cfg", b"/home/nvidia/darknet/yolov3-tiny.weights", 0)
        meta = darknet.load_meta(b"/home/nvidia/darknet/cfg/coco.data")

        # cv2.namedWindow("cv2_img", cv2.WINDOW_NORMAL)

        while not rospy.is_shutdown():
            if self.last_img is not None:
                self.do_stuff(net,meta)
            r.sleep()


def main():
    rospy.init_node('camera_test')
    ocvs = OpenCVSkeleton()
    ocvs.run()
    # net = darknet.load_net(b"/home/usl/darknet/cfg/yolov2.cfg", b"/home/usl/darknet/yolov2.weights", 0)
    # meta = darknet.load_meta(b"/home/usl/darknet/cfg/coco.data")
    # cv2.namedWindow("img", cv2.WINDOW_NORMAL)

    # while not rospy.is_shutdown():
    #     if ocvs.is_new_img:
    #         r = detect_np(net, meta, img)
    #         # print(r)
    #         for i in r:
    #             x, y, w, h = i[2][0], i[2][1], i[2][2], i[2][3]
    #             xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
    #             pt1 = (xmin, ymin)
    #             pt2 = (xmax, ymax)
    #             cv2.rectangle(img, pt1, pt2, (0, 255, 0), 2)
    #             cv2.putText(img, i[0].decode() + " [" + str(round(i[1] * 100, 2)) + "]", (pt1[0], pt1[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 0], 4)
    #             cv2.imshow("img", img)
    #             k = cv2.waitKey(1)
    #             if k == 27:
    #                 cv2.destroyAllWindows()
    #                 exit()
    rospy.spin()
