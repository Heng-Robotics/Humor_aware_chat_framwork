#!/usr/bin/env python
# coding:utf-8

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import shutil
from utils.config import PathConfig

save_image_flag = False  # 用于控制是否保存图像

def clear_image_directory():
    if os.path.exists(PathConfig.image_dir):
        shutil.rmtree(PathConfig.image_dir)  # 删除文件夹及其内容
    os.makedirs(PathConfig.image_dir)  # 重新创建文件夹

def take_picture_callback(msg):
    global save_image_flag
    if msg.data:
        # rospy.loginfo("Start saving images. Clearing directory first.")
        clear_image_directory()  # 清空保存图片的文件夹
        save_image_flag = True

def stop_picture_callback(msg):
    global save_image_flag
    if msg.data:
        # rospy.loginfo("Stop saving images.")
        save_image_flag = False
        process_emotion_pub.publish(True)

def publish_image():
    pub = rospy.Publisher('/camera_raw', Image, queue_size=1)
    global process_emotion_pub
    process_emotion_pub = rospy.Publisher('/process_emotion', Bool, queue_size=1)
    
    rospy.Subscriber('/take_picture_flag', Bool, take_picture_callback)
    rospy.Subscriber('/stop_picture_flag', Bool, stop_picture_callback)

    rate = rospy.Rate(30)  
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)  

    image_count = 0  # 用于保存图像时的计数

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        cv2.imshow('image', frame)
        cv2.waitKey(1)

        if ret:
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            except CvBridgeError as e:
                rospy.logerr("Error converting image", str(e))
                continue

            pub.publish(ros_image)
            rate.sleep()

            if save_image_flag:
                image_filename = PathConfig.image_dir + "image_" + str(image_count) + ".jpg"
                cv2.imwrite(image_filename, frame)
                image_count += 1
                time.sleep(1)  # 每秒保存一张图像

    cap.release()
    cv2.destroyAllWindows()
    rospy.loginfo("Image publisher node stopped.")

if __name__ == '__main__':
    rospy.init_node('camera', anonymous=True)
    rospy.loginfo("Camera node initialized.")
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
