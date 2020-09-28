#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
import std_msgs.msg as std
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge



global ax
global ay
global d_max
global cut_idx

ax = [-0.06, 1.0, -0.6, 14.0]
ay = [-0.17, 2.9, -3.6, 4.0]
d_max = 10
cut_idx = [65, 605, 15, 1095]

def info_param():

    data = CameraInfo()

    data.height = cut_idx[2] - cut_idx[1]
    data.width = cut_idx[4] - cut_idx[3]
    data.distortion_model = "plumb_bob"
    data.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    data.K = [1324.1, 0.0, 808.6, 0.0, 1324.1, 171.7, 0.0, 0.0, 1.0]
    data.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    data.P = [1324.1, 0.0, 808.6, 0.0, 0.0, 1324.1, 171.7, 0.0, 0.0, 0.0, 1.0, 0.0]
    data.binning_x = 0
    data.binning_y = 0
    data.roi.x_offset = 0
    data.roi.y_offset = 0
    data.roi.height = 0
    data.roi.width = 0
    data.roi.do_rectify = False

    return data



def image_publisher():

    rospy.init_node('video_to_image', anonymous=True)

    mono_pub = rospy.Publisher("/video/mono/image", Image, queue_size=10)
    mono_raw_pub = rospy.Publisher("/video/mono/image_raw", Image, queue_size=10)
    mono_info_pub = rospy.Publisher("/video/mono/camera_info", CameraInfo, queue_size=10)

    depth_pub = rospy.Publisher("/video/depth/image", Image, queue_size=10)
    depth_raw_pub = rospy.Publisher("/video/depth/image_raw", Image, queue_size=10)
    depth_info_pub = rospy.Publisher("/video/depth/camera_info", CameraInfo, queue_size=10)

    swreg_pub = rospy.Publisher("/video/swreg/image", Image, queue_size=10)
    swreg_raw_pub = rospy.Publisher("/video/swreg/image_raw", Image, queue_size=10)
    swreg_info_pub = rospy.Publisher("/video/swreg/camera_info", CameraInfo, queue_size=10)

    filename = sys.argv[1]
    video = cv2.VideoCapture(filename)

    if not video.isOpened():
        print "Error: Can't open: " + str(filename)
        exit(0)
    print "Correctly opened, starting to publish."

    fps = video.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)
    count_max = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

    bridge = CvBridge()

    header_msg = std.Header()
    header_msg.frame_id = "/video_link"

    rgb_info = info_param()
    depth_info = info_param()
    swreg_info = info_param()

    success, frame = video.read()

    count = 1

    while success and not rospy.is_shutdown():

        rgb_img = frame[cut_idx[1]:cut_idx[1], cut_idx[3]:cut_idx[3]]
        mono_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
        mono_filt_img = cv2.GaussianBlur(mono_img, (23,23), 0)
        depth_img = 100.0 / (mono_filt_img + 100.0/d_max)
        depth_raw_img = 1000.0 * depth_img

        mono = bridge.cv2_to_imgmsg(np.uint8(mono_img), encoding="mono8")
        mono_raw = bridge.cv2_to_imgmsg(np.uint8(mono_img), encoding="16UC1")
        depth = bridge.cv2_to_imgmsg(np.float32(depth_img), encoding="32FC1")
        depth_raw = bridge.cv2_to_imgmsg(np.uint16(depth_raw_img), encoding="16UC1")
        swreg = bridge.cv2_to_imgmsg(np.float32(depth_img), encoding="32FC1")
        swreg_raw = bridge.cv2_to_imgmsg(np.uint16(depth_raw_img), encoding="16UC1")

        header_msg.stamp = rospy.Time.now()
        header_msg.seq = count

        mono.header = header_msg
        mono_raw.header = header_msg
        mono_info.header = header_msg
        depth.header = header_msg
        depth_raw.header = header_msg
        depth_info.header = header_msg
        swreg.header = header_msg
        swreg_raw.header = header_msg
        swreg_info.header = header_msg

        mono_pub.publish(mono)
        mono_raw_pub.publish(mono_raw)
        mono_info.publish(mono_info)
        depth_pub.publish(depth)
        depth_raw_pub.publish(depth_raw)
        depth_info_pub.publish(depth_info)
        swreg_pub.publish(swreg)
        swreg_raw_pub.publish(swreg_raw)
        swreg_info_pub.publish(swreg_info)

        print str(count) + "/" + str(count_max)

        rate.sleep()

        success, frame = video.read()
        count += 1



if __name__ == "__main__":
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
