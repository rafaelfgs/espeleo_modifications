#!/usr/bin/env python

import rosbag
import rospy
import sys
import os

input_file = "/media/rafael/Seagate Expansion Drive/2020_11_COPPELIA/cave_average.bag"
output_file = "/media/rafael/Seagate Expansion Drive/2020_11_COPPELIA/cave_short.bag"

start_time = 1933.0
duration_time = 255.0

if __name__ == "__main__":
    
    rospy.sleep(1.0)
    
    sys.stdout.write("\n%s\n" % input_file)
    sys.stdout.flush()
    
    if not os.path.exists(input_file):
        sys.stdout.write("File not found.\n")
        sys.stdout.flush()
        sys.exit()
    
    if os.path.exists(output_file):
        if raw_input("Output file already exists, do you want to replace it? (y/n): ") == "y":
            os.remove(output_file)
        else:
            sys.stdout.write("No changes made.\n")
            sys.stdout.flush()
            sys.exit()
    
    sys.stdout.write("\nOpening file...\n")
    sys.stdout.flush()
    
    bag_in = rosbag.Bag(input_file)
    
    start_time = start_time + bag_in.get_start_time()
    end_time = duration_time + start_time
    
    if start_time > bag_in.get_end_time():
        sys.stdout.write("\nNo changes made.\n")
        sys.stdout.flush()
        sys.exit()
        
    elif end_time > bag_in.get_end_time():
        sys.stdout.write("\nDuration time extrapolates bag time, it will be truncated.\n")
        sys.stdout.flush()
        duration_time = bag_in.get_end_time() - start_time
        end_time = bag_in.get_end_time()
    
    sys.stdout.write("\nBag starting: %.9f"  % start_time)
    sys.stdout.write("\nBag ending:   %.9f"  % end_time)
    sys.stdout.write("\nBag duration: %.1fs" % duration_time)
    sys.stdout.flush()
    
    bag_out = rosbag.Bag(output_file, "w")
    
    sys.stdout.write("\n\nStarting to correct bag...\n\n")
    sys.stdout.flush()
    
    for topic, msg, t in bag_in.read_messages():
        
        rospy.sleep(1e-4)
        
        if rospy.Time.to_sec(t) > start_time and rospy.Time.to_sec(t) < end_time:
            bag_out.write(topic, msg, t)
        
        status_time = 100.0 * ( (rospy.Time.to_sec(t) - bag_in.get_start_time()) / 
                                (bag_in.get_end_time() - bag_in.get_start_time()) )
                                
        sys.stdout.write("\rRepublishing topics... %5.1f%%" % status_time)
        sys.stdout.flush()
    
    sys.stdout.write("\n\nFile cutted with %.2fGB" % (bag_out.size/1073741824.0))
    
    bag_in.close()
    bag_out.close()
    
    sys.stdout.write("\n\nFiles Closed\n")
    sys.stdout.flush()