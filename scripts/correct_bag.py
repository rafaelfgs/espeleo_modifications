#!/usr/bin/env python

import rospy
import rosbag
import os
import sys
from copy import copy



input_file = "/mnt/WD500/UFMG/STIX/cave-rgbd-test_2020-08-12-18-53-21.bag"
output_file = input_file[:-4] + "_corrected.bag"



def main_function():
    
    global bag_in, bag_out, topic, msg, t

    rospy.init_node("correct_bag_node", anonymous=True)
    rospy.sleep(0.1)
    
    print("\n%s" % input_file)
    rospy.sleep(0.1)
    
    if raw_input("Are you sure this is the correct file? (y/n): ") == "y":
        print("\n%s" % output_file)
        rospy.sleep(0.1)
        if os.path.exists(output_file):
            if raw_input("Output file already exists, do you want to replace it? (y/n): ") == "y":
                os.remove(output_file)
            else:
                sys.exit(0)
    else:
        sys.exit(0)
    
    rospy.sleep(0.1)
    print("\nOpening bag...\n")
    bag_in = rosbag.Bag(input_file)
    bag_out = rosbag.Bag(output_file, "w")
    
    for topic, msg, t in bag_in.read_messages():
        
        if rospy.is_shutdown():
            break
        
        if hasattr(msg, "header"):
            msg.header.stamp = copy(t)
        
        if topic == "/tf":
            msg.transforms[0].header.stamp = copy(t)
        
        bag_out.write(topic, msg, t)
        
        status_time = 100.0 * (rospy.Time.to_sec(t) - bag_in.get_start_time()) / (bag_in.get_end_time() - bag_in.get_start_time())
        sys.stdout.write("\rPublishing %-34s %5.1f%%" % (topic, status_time))
        sys.stdout.flush()
        rospy.sleep(0.001)
    
    bag_in.close()
    bag_out.close()



if __name__ == "__main__":
    main_function()