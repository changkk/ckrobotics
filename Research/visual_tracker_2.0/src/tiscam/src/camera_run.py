import tiscamera
import rospy
import numpy as np
import time
import math
from std_msgs.msg import Float64, Int8
from geometry_msgs.msg import PolygonStamped, Point32


#----------------------------------------------------------------------------------------------------------------------------------
# Open the camera. Parameters are serial number, width, height, frame rate, color and liveview.
cam = tiscamera.Camera("50710011", 1280, 960, 30, True, False)

# Start the live stream from the camera and also "rosrun"
cam.start_pipeline()

# Set some properties
# cam.set_property("Exposure Auto", True)
cam.set_property("Exposure", 300000)

cam.set_property("Gain Auto", True)
# cam.set_property("Brightness Reference", 128)
# cam.set_property("Brightness Reference", 5000)
cam.set_property("Focus Auto", True)
# cam.set_property("Focus", 1000)

#cam.PushProperty("Focus Auto")
#----------------------------------------------------------------------------------------------------------------------------------


zoom = 0
zm = 0
range = 0
flag = 0
meas_flag = False
manual_control = False

def rounddown(x):
    return int(math.floor(x / 1.0)) * 1

# def rangeCallback(msg):
#     global range
#     range = msg.data
#     global zoom
#     # zoom = sensor_size*range*prcnt_coverage/target_size
#     zoom = 0.00626*range*0.1/1

#     if zoom <= 0.0048:
#         zoom = 0
#     elif zoom >= 0.0576:
#         zoom = 100
#     else:
#         zoom = rounddown(1893.93*zoom-6.09)
    

def focusCallback(msg):
    global flag
    flag = 1

# Bounding box pixel size callback
def BBSCallback(msg):

    if ~manual_control:
        global BBS
        global meas_flag
        global zoom
        global zm

        img_size = msg.polygon.points[1]
        BBS = msg.polygon.points[2]

        if BBS.x > BBS.y:
            img_size = img_size.x
            BBS = BBS.x
        else:
            img_size = img_size.y
            BBS = BBS.y

        # current_focal = (zoom*0.528 + 4.8) / 1000

        # desired_focal_length = (resolution*percent)_coverage / BoundingBoxSize * current_focal_length
        # desired_focal = (img_size*0.5) / BBS * current_focal

        # zoom = sensor_size*range*prcnt_coverage/target_size
        # zoom = 0.00626*range*0.1/1

        box_ratio = BBS/img_size
        # prcnt_coverage = 0.10
        prcnt_coverage = 0.10

        #This should either be in a control loop operating on a filtered state or a mapping function.
        #Right now it tends to oscillate zoom cause of our zoom step restrictions (rounding to closest ten).
        #ALSO - add a delay between initial detection and zooming; right now the camera zooms in right away if the box_ratio is low
        # and if the aircraft isn't already centered in frame, the camera loses it the moment it zooms in - fail. Observe offset bw
        # pid state and setpoint then make this call.
        if box_ratio > prcnt_coverage:
            zm -= 1
        else:
            zm += 1

        zoom = zm

        if zoom < 0:
            zoom = 0
            zm = 0
        elif zoom > 100:
            zoom = 100
            zm = 100
        else:
            zoom = rounddown(zm)
            # zoom = round((desired_focal*1000-4.8)/0.528)
            # zoom_my = rounddown(1893.93*desired_focal-6.09)

        meas_flag = True

        # print 'BoxSize = ', BBS, '   ImgSize = ', img_size
        # print 'Zoom = ', zoom
    
def JoystickCallback(msg):
    global zoom
    global manual_control
    
    manual_control = msg.data[1]
    zm = msg.data[0]

    zoom = zoom + zm*2


def main():
    rospy.init_node('camera_node', anonymous=True)

    pub_zoom = rospy.Publisher("zoom", Float64, queue_size=1)
    # rospy.Subscriber("range", Float64, rangeCallback)
    rospy.Subscriber("focus", Float64, focusCallback)
    rospy.Subscriber("yolo_detection_box", PolygonStamped, BBSCallback)
    rospy.Subscriber("man_zoom", Int8, JoystickCallback)


    rate = rospy.Rate(2)
    global zoom
    global range
    global flag
    zoom = 0
    z_old = 10
    no_meas = 0
    focus_counter = 0

    while not rospy.is_shutdown():

        #--------------------------------------------------------------
        # zoom = 0
        
        if zoom != z_old:
            cam.set_property("Zoom", zoom)
            time.sleep(0.001)
            # str = "Zoom: %d Range: %f" % (zoom, range)
            str = "Zoom: %d" % (zoom)
            print str
            cam.push_property("Focus Auto")
            z_old = zoom
            no_meas = 0
            
        else:
            no_meas += 1
        
        if flag == 1:
            cam.push_property("Focus Auto")
            time.sleep(0.001)
            flag = 0
            print 'Adjusting Focus'

        if no_meas > 10:
            zoom = 0

        #--------------------------------------------------------------

        if focus_counter > 10:
            cam.push_property("Focus Auto")
            focus_counter = 0
        else:
            focus_counter ++ 1

        pub_zoom.publish(zoom)
        rate.sleep()

    #Stop the camera pipeline.
    cam.stop_pipeline() #----------------------------------------
    print('Program ended')

# input("Change a property by hitting enter.")

# #cam.list_properties()

# cam.set_property("Zoom", 100)
# input("Change a property again by hitting enter.")
# cam.push_property("Focus Auto")
# input("Change a property again by hitting enter.")

# cam.set_property("Zoom", 0)

# input("Press Enter to end program")

# # Stop the camera pipeline.
# cam.stop_pipeline()
# print('Program ended')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
