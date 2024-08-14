#!/usr/bin/env python

import os
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from datetime import datetime, timedelta

def extract_images_from_bag(bag_file, output_dir, topic):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    frame_count = 0
    saved_count = 0
    total_frames = bag.get_message_count(topic_filters=[topic])
    
    if total_frames < 10:
        frame_skip = total_frames  # Save only 1 frame
    elif total_frames < 100:
        frame_skip = 20  # Save every 10th frame
    elif total_frames < 1000:
        frame_skip = 100  # Save every 50th frame
    else:
        frame_skip = 500  # Save every 250th frame

    bag_name = os.path.splitext(os.path.basename(bag_file))[0]
    
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        if frame_count % frame_skip == 0:
            # Convert the compressed image message to an OpenCV image
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # Generate a filename and save the image
            output_file = os.path.join(output_dir, f"{bag_name}_frame_{saved_count:06d}.jpg")
            cv2.imwrite(output_file, cv_image)
            saved_count += 1
            
        frame_count += 1
    
    bag.close()
    print(f"Processed {frame_count} frames, saved {saved_count} images to {output_dir}")

def parse_bag_start_time(bag_file):
    # Assume filename format is aqc_808_YYYY-MM-DD-HH-MM-SS.bag
    base_name = os.path.basename(bag_file)
    time_str = base_name.split('_')[-1].split('.')[0]  # Extract time part and remove '.bag' extension
    return datetime.strptime(time_str, "%Y-%m-%d-%H-%M-%S")

if __name__ == "__main__":
    # Directory containing ROS bag files
    bag_directory = "week1_0806"
    # Directory where images will be saved
    output_directory = "/home/yyp/PSA/data_0806_suite1_8/images_5mins"
    # ROS topic to extract images from
    topic_name = "/suite_1/8/rgb/compressed"
    
    last_bag_time = None
    for bag_file in sorted(os.listdir(bag_directory)):
        if bag_file.endswith(".bag"):
            bag_file_path = os.path.join(bag_directory, bag_file)
            bag_start_time = parse_bag_start_time(bag_file_path)
            
            if last_bag_time is None or (bag_start_time - last_bag_time) >= timedelta(minutes=5):
                extract_images_from_bag(bag_file_path, output_directory, topic_name)
                last_bag_time = bag_start_time
