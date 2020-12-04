#!/usr/bin/env python

import rosbag
import rospy
import cv2
import sys
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


input_file = "/mnt/WD500/UFMG/DISSERTACAO/bags/veloso_0000_120090507380.bag"
output_path = "/mnt/WD500/UFMG/DISSERTACAO/bags/veloso_img/"


if __name__ == "__main__":
    
    rospy.sleep(1.0)
    
    print("\n%s" % input_file)    
    
    if not os.path.exists(input_file):
        print("File not found.\n")
        sys.exit()
    
    if os.listdir(output_path):
        if raw_input("Output path is not empty, do you want to replace it? (y/n): ") == "y":
            output_list = [os.remove(output_path+x) for x in os.listdir(output_path)]
        else:
            print("No changes made.\n")
            sys.exit()
    
    print("\nOpening file...")
    bag_in = rosbag.Bag(input_file)
    
    print("\nStarting to save image messages...\n")
    
    for topic, msg, t in bag_in.read_messages():
        
        rospy.sleep(1e-4)
        current_time = rospy.Time.to_sec(t)
        
        if rospy.is_shutdown():
            k = len(input_file)
            break
            
        elif topic == "/d435i/color/image_raw/compressed":
            output_name = "color_%d_%d.jpg" % (current_time,(1e9*(current_time%1)))
            img_color = CvBridge().compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(output_path + output_name, img_color)
            
        elif topic == "/d435i/aligned_depth_to_color/image_raw/compressedDepth":
            output_name = "depth_%d_%d.png" % (current_time,(1e9*(current_time%1)))
            msg_depth = CompressedImage()
            msg_depth.format = "png"
            msg_depth.data = msg.data[12:]
            img_depth = 8 * CvBridge().compressed_imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
            cv2.imwrite(output_path + output_name, img_depth)
                
        status_time = 100.0 * ( (current_time - bag_in.get_start_time()) / (bag_in.get_end_time() - bag_in.get_start_time()) )
        print("\rSaving images... %5.1f%%" % status_time),
        
    print("\n\nCreated %d image files" % (len(os.listdir(output_path))))
    
    bag_in.close()
    print("\nBag file closed\n")