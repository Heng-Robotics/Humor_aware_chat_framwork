#!/usr/bin/env python
# coding:utf-8
import os
import cv2
import numpy as np
import rospy
from utils.config import PathConfig
from utils.config_robot import RobotConfig
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PepperCamera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub=rospy.Publisher('/image_view/image_raw', Image, queue_size = 1) 
        self.image = None
        self.ros_image = None
        self.fps = RobotConfig.fps
        self.camera = RobotConfig.camera
        self.resolution = RobotConfig.resolution
        self.color_space = RobotConfig.color_space        
        self.camera_device = RobotConfig.camera_device

        self.camera_index = None
        if self.camera == "camera_top":
            self.camera_index = 0
        elif self.camera == "camera_bottom":
            self.camera_index = 1
        elif self.camera == "camera_depth":
            self.camera_index = 2

        self.camera_link = self.camera_device.subscribeCamera("Camera_Stream" + str(np.random.random()),
                                                              self.camera_index, self.resolution, self.color_space, self.fps)
        rospy.loginfo("pepper_camera node initialized.")

    def subscribe_camera(self):
        """
        :param camera: `camera_depth`, `camera_top` or `camera_bottom`
        :camera: string
        :param resolution:
            0. 160x120
            1. 320x240
            2. 640x480
            3. 1280x960
        :type resolution: integer
        :param fps: Frames per sec (5, 10, 15 or 30)
        :type fps: integer
        """

        image_raw = self.camera_device.getImageRemote(self.camera_link)
        self.image = np.frombuffer(image_raw[6], np.uint8).reshape(image_raw[1], image_raw[0], 3)

        cv2.imshow("Pepper Camera", self.image)
        cv2.waitKey(1)

        self.ros_image = self.bridge.cv2_to_imgmsg(self.image, "bgr8") 
        self.image_pub.publish(self.ros_image)

def init_folder(path):
    for filename in os.listdir(path):
        file_path = os.path.join(path, filename)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                os.rmdir(file_path)
        except:
            print("can't delete")


if __name__=="__main__":

    rospy.init_node('pepper_camera', anonymous=True) 
    pepper_camera = PepperCamera()

    init_folder(PathConfig.face_img_folder)
    init_folder(PathConfig.captioning_img_folder)
    init_folder(PathConfig.json_folder_path)

    
    while not rospy.is_shutdown():
        pepper_camera.subscribe_camera()
        # rospy.loginfo("pepper image")

    
