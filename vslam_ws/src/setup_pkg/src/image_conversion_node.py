#! /usr/bin/python3

# Node for extracting images from a rosbag in the
# format needed for ORB_SLAM3.
# Run these commands first: (images will go here)
#   mkdir -p big_data/DATASET_NAME/images/mav0/cam0/data
#   mkdir -p big_data/DATASET_NAME/images/mav0/cam1/data

# Referenced https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
# Topic names.
left_cam_topic = "/camera_array/cam0/image_raw"
right_cam_topic = "/camera_array/cam1/image_raw"
# Image save location.
images_filepath = "big_data/car_path2/"
# Ensure images have time correspondance.
timestamps = []
cam1_index = -1

def get_cam0(msg):
    global timestamps
    # cam0 will control the time.
    time = msg.header.stamp
    timestamps.append(time)
    # write the next timestamp to the file.
    timestamps_file.write(str(time)+"\n")
    try:
        # Convert the Image msg to OpenCV2 object.
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8") #bgr8
        # Save image with time as name.
        cv2.imwrite(images_filepath+"images/mav0/cam0/data/"+str(time)+'.png', cv2_img)
        # rospy.sleep(0.05)
    except:
        print("Exception encountered on cam0.")

def get_cam1(msg):
    global cam1_index
    # use the time from cam0.
    cam1_index += 1
    # wait if necessary until cam0 has come in.
    time = None
    while len(timestamps)-1 < cam1_index:
        rospy.sleep(0.05)
    try:
        time = timestamps[cam1_index]
    except:
        print(len(timestamps), cam1_index)
    
    try:
        # Convert the Image msg to OpenCV2 object.
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8") #bgr8
        # Save image with time as name.
        cv2.imwrite(images_filepath+"images/mav0/cam1/data/"+str(time)+'.png', cv2_img)
        # rospy.sleep(0.05)
    except:
        print("Exception encountered on cam1.")

def main():
    global timestamps_file
    rospy.init_node('image_conversion_node')
    # Init file for timestamps.
    timestamps_file = open(images_filepath+"/timestamps.txt", "w")
    # Subscribe to the image streams.
    rospy.Subscriber(left_cam_topic, Image, get_cam0)
    rospy.Subscriber(right_cam_topic, Image, get_cam1)
    rospy.spin()

if __name__ == '__main__':
    main()