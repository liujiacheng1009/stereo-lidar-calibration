# Start up ROS pieces.
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d

# Reading bag filename from command line or roslaunch parameter.
import os
import sys

class ImageCreator():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self, bagfiles):
        # Get parameters when starting node from a launch file.
        save_dir_left = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/mydata/left_images"
        save_dir_right = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/mydata/right_images"
        # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
        self.bridge = CvBridge()
        index = 1
        for filename in bagfiles:
        # Open bag file.
            rospy.loginfo("Bag filename = %s", filename)
            flag_l = False
            flag_r = False
            index_str = str(index).zfill(4)
            with rosbag.Bag(filename, 'r') as bag:
                for topic, msg, t in bag.read_messages():
                    if topic == "/stereo_camera/left/image_rect_color":
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print(e)
                        image_name = str(save_dir_left)+"/"+index_str+".png"
                        cv2.imwrite(image_name, cv_image)
                        flag_l = True
                    if topic == "/stereo_camera/right/image_rect_color":
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print(e)
                        image_name = str(save_dir_right)+"/"+index_str+".png"
                        cv2.imwrite(image_name, cv_image)
                        flag_r = True
                    if flag_l and flag_r:
                        index += 1
                        break


# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("bag_reader")
    # Go to class functions that do all the heavy lifting. Do error checking.
    bagfiles = []
    bag_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/mydata"
    f_list = os.listdir(bag_path)
    # print f_list
    for i in f_list:
        # os.path.splitext():分离文件名与扩展名
        if os.path.splitext(i)[1] == '.bag':
            bagfiles.append(bag_path +"/"+ i)
    bagfiles.sort()
    print(bagfiles)
    try:
        image_creator = ImageCreator(bagfiles)
    except rospy.ROSInterruptException: pass