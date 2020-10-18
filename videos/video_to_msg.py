#!/usr/bin/env python

import rospy
import cv2
import sys
import time
import numpy as np
import std_msgs.msg as std
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge



def info_480p():

    data = CameraInfo()

    data.height = 480
    data.width = 640

    data.distortion_model = "plumb_bob"

    data.D = [ -0.297117,   0.070173,  -0.000390,   0.005232,   0.000000]

    data.K = [342.399061,   0.000000, 284.222700,
                0.000000, 343.988302, 227.555237,
                0.000000,   0.000000,   1.000000]

    data.R = [  1.000000,   0.000000,   0.000000,
                0.000000,   1.000000,   0.000000,
                0.000000,   0.000000,   1.000000]

    data.P = [247.555710,   0.000000, 290.282918,   0.000000,
                0.000000, 281.358612, 220.776009,   0.000000,
                0.000000,   0.000000,   1.000000,   0.000000]

    data.binning_x = 0
    data.binning_y = 0
    data.roi.x_offset = 0
    data.roi.y_offset = 0
    data.roi.height = 0
    data.roi.width = 0
    data.roi.do_rectify = False

    return data



def info_720p():

    data = CameraInfo()

    data.height = 720
    data.width = 1280

    data.distortion_model = "plumb_bob"

    data.D = [ -0.266169,   0.056566,   0.003569,   0.002922,   0.000000]

    data.K = [550.515474,   0.000000, 560.486530,
                0.000000, 550.947091, 334.377088,
                0.000000,   0.000000,   1.000000]

    data.R = [  1.000000,   0.000000,   0.000000,
                0.000000,   1.000000,   0.000000,
                0.000000,   0.000000,   1.000000]

    data.P = [395.931458,   0.000000, 570.158643,   0.000000,
                0.000000, 474.049377, 329.846866,   0.000000,
                0.000000,   0.000000,   1.000000,   0.000000]

    data.binning_x = 0
    data.binning_y = 0
    data.roi.x_offset = 0
    data.roi.y_offset = 0
    data.roi.height = 0
    data.roi.width = 0
    data.roi.do_rectify = False

    return data



def video_to_msg():

    rospy.init_node("video_to_msg_node", anonymous=True)

    pub_image = rospy.Publisher("/axis/image_raw", Image, queue_size=10)
    pub_compr = rospy.Publisher("/axis/image_raw/compressed", CompressedImage, queue_size=10)
    pub_info = rospy.Publisher("/axis/camera_info", CameraInfo, queue_size=10)

    filename = sys.argv[1]
    video = cv2.VideoCapture(filename)

    if not video.isOpened():
        print "Error: Can't open: " + str(filename)
        exit(0)
    print "Correctly opened, starting to publish."

    fps = video.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)
    count_max = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    resolution = video.get(cv2.CAP_PROP_FRAME_HEIGHT)

    bridge = CvBridge()

    msg_header = std.Header()
    msg_header.frame_id = "/axis"

    if resolution == 720.0:
        msg_info = info_720p()
    elif resolution == 480.0:
        msg_info = info_480p()

    success, frame = video.read()
    count = 1

    while success and not rospy.is_shutdown():
        
        msg_image = bridge.cv2_to_imgmsg(np.uint8(frame), encoding="rgb8")
        msg_compr = bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg")

        msg_header.stamp = rospy.Time.now()
        msg_header.seq = count

        msg_image.header = msg_header
        msg_compr.header = msg_header
        msg_info.header = msg_header
        #print msg_compr
        #time.sleep(10)

        pub_image.publish(msg_image)
        pub_compr.publish(msg_compr)
        pub_info.publish(msg_info)

        print str(count) + "/" + str(count_max)

        rate.sleep()

        success, frame = video.read()
        count += 1



if __name__ == "__main__":
    try:
        video_to_msg()
    except rospy.ROSInterruptException:
        pass
