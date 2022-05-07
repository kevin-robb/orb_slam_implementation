#! /usr/bin/python3

# Node for extracting images from a rosbag in the format needed for ORB_SLAM3.
# Referenced https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import sys

bridge = CvBridge()
# folder name for dataset
IMAGES_PATH = None
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
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Save image with time as name.
        cv2.imwrite(IMAGES_PATH+"images/mav0/cam0/data/"+str(time)+'.png', cv2_img)
    except:
        rospy.logerr("Exception encountered on cam0.")

def get_cam1(msg):
    global cam1_index
    # use the time from cam0.
    cam1_index += 1
    # wait if necessary until cam0 has come in for this timestep.
    time = None
    while len(timestamps)-1 < cam1_index:
        rospy.sleep(0.05)
    try:
        time = timestamps[cam1_index]
    except:
        rospy.logerr("cam1 unable to sync with cam0 at timestamp "+str(len(timestamps))+", index "+str(cam1_index))
    
    try:
        # Convert the Image msg to OpenCV2 object.
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Save image with time as name.
        cv2.imwrite(IMAGES_PATH+"images/mav0/cam1/data/"+str(time)+'.png', cv2_img)
    except:
        rospy.logerr("Exception encountered on cam1.")

def run_bash_cmd(command:str):
    # run something on the command line.
    process = subprocess.Popen(command.split())
    output, error = process.communicate()

def main():
    global timestamps_file, IMAGES_PATH
    rospy.init_node('image_conversion_node')

    # get the dataset name and topics from cmd line params.
    if len(sys.argv) > 2:
        DATASET_NAME = sys.argv[1]
        LEFT_CAM_TOPIC = sys.argv[2]
    else:
        rospy.logerr("Must provide DATASET_NAME and LEFT_CAM_TOPIC params.")
        exit()
    # right cam is optional, and is unused for monocular setups.
    RIGHT_CAM_TOPIC = sys.argv[3] if len(sys.argv) > 3 else "/NONE"

    # create the folder images will be saved to. starts from ~/.ros
    IMAGES_PATH = "../Coursework/eece5554-vslam/big_data/"+DATASET_NAME
    run_bash_cmd("mkdir -p "+IMAGES_PATH+"/images/mav0/cam0/data")
    run_bash_cmd("mkdir -p "+IMAGES_PATH+"/images/mav0/cam1/data")
    rospy.loginfo("Images will be saved to "+IMAGES_PATH+"/images")
    # Init file for timestamps.
    timestamps_file = open(IMAGES_PATH+"/timestamps.txt", "w")
    
    # Subscribe to the image streams.
    rospy.Subscriber(LEFT_CAM_TOPIC, Image, get_cam0)
    rospy.Subscriber(RIGHT_CAM_TOPIC, Image, get_cam1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass