#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
这个节点会对机器人进行以下控制
1. 控制机器人在启动之后进行左右张望的动作;
2. 控制机器人进行拍摄图片:在右.左.中视角下分别拍摄一张图片用来做captioning. 
     此外, 还会在看向正中间时拍下一张照片用来做deepface分析
3. 控制机器人播放音频；

这个ROS节点中会订阅来自机器人相机和RGB相机的消息,
以及来自captioning节点再次拍摄照片的请求消息.
以及将htchat内容以音频形式播放的请求.
"""

import rospy
import time
import cv2
import numpy as np
from utils.config import PathConfig
from utils.config_robot import RobotConfig
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class robot_behavior():
    def __init__(self):
        self.image = None
        self.camera = None
        self.bridge = CvBridge()
        self.face_pic_flag = String()
        self.face_pic_flag.data = 'new_picture_coming'

        self.image_sub = rospy.Subscriber('/image_view/image_raw', Image, self.image_callback)
        self.camera_sub = rospy.Subscriber('/camera_raw', Image, self.camera_callback)
        self.context_sub= rospy.Subscriber("/deepface_flag", String, self.deepface_callback)
        # self.audio_sub = rospy.Subscriber("/play_audio", String, self.audio_callback)
        self.tts_sub = rospy.Subscriber("/speak_content", String, self.tts_callback)
        self.beep_sub = rospy.Subscriber("/beep_flag", Bool, self.beep_callback)      

        self.picture_start_pub = rospy.Publisher("/take_picture_flag", Bool, queue_size=1)
        self.picture_stop_pub = rospy.Publisher("/stop_picture_flag", Bool, queue_size=1)
        self.recognition_pub = rospy.Publisher("/recognition_flag", Bool, queue_size=1)
        self.face_pic_flag_pub = rospy.Publisher("/context", String, queue_size=1)
        self.talk_start_pub = rospy.Publisher("/start_talk_flag", String, queue_size=1)

        self.motion_service = RobotConfig.motion_service
        self.posture_service = RobotConfig.posture_service
        self.detection_service = RobotConfig.detection_service
        
        self.start_talk_msg = String()
        self.start_talk_msg.data = "talk"
        self.look_around_flag = True
        self.speech_recognition_msg = Bool()
        self.speech_recognition_flag = True
        self.picture_flag_msg = Bool()
        self.picture_flag = True

    def beep_callback(self, msg):
        try:
            if msg.data == True: 
                time.sleep(0.95)  # Wait for 0.5 second
                RobotConfig.leds_service.on("ChestLeds")
                RobotConfig.audio_service.playSine(400, 100, 0, 0.2)  # Play a 1000 Hz sine wave for 0.5 second
                time.sleep(0.5)  # Wait for 0.5 second
                RobotConfig.leds_service.off("ChestLeds")
                time.sleep(0.5)  # Wait for 0.5 second
        except Exception as e:
            rospy.logerr("There is an error in robot_behavior/beep_callback: " + str(e))

    def camera_callback(self, msg):
        try:
            self.camera = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("There is an error in robot_behavior/camera_callback: " + str(e))

    def tts_callback(self, msg):
        self.speech_recognition_msg.data = self.speech_recognition_flag
        self.recognition_pub.publish(self.speech_recognition_msg.data)
        try:
            self.picture_start_pub.publish(self.picture_flag)#######
            if msg.data is not None: #当消息不为空的时候，就使用TTS播放
                RobotConfig.animated_speech_service.say(msg.data, RobotConfig.configuration)

                # self.speech_recognition_flag = False
            self.picture_stop_pub.publish(self.picture_flag) ########

        except Exception as e:
            rospy.logerr("There is an error in robot_behavior/tts_callbcak: " + str(e))
          

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("There is an error in robot_behavior/image_callback: " + str(e))


    def deepface_callback(self, msg):
        try:
            if msg.data == 'face_not_found':
                self.find_face()#take photo again
                self.face_pic_flag_pub.publish(self.face_pic_flag.data)
                rospy.loginfo("take a new photo with face!")
        except Exception as e:
            rospy.logerr("There is an error in robot_behavior/deepface_callback: " + str(e))

    def look_around(self):

        if self.look_around_flag ==True:

            L_data_len = len(RobotConfig.L_gesture_list)
            for i in np.arange(L_data_len):
                self.motion_service.setAngles(RobotConfig.joint_name, [RobotConfig.L_gesture_list[i][0], RobotConfig.L_gesture_list[i][1], RobotConfig.L_gesture_list[i][2], RobotConfig.L_gesture_list[i][3], RobotConfig.L_gesture_list[i][4], RobotConfig.L_gesture_list[i][5],
                                              RobotConfig.L_gesture_list[i][6], RobotConfig.L_gesture_list[i][7], RobotConfig.L_gesture_list[i][8], RobotConfig.L_gesture_list[i][9], RobotConfig.L_gesture_list[i][10],
                                              RobotConfig.L_gesture_list[i][11], RobotConfig.L_gesture_list[i][12], RobotConfig.L_gesture_list[i][13]], 0.3)                  
                time.sleep(0.025)

            time.sleep(0.5)
            cv2.imwrite(PathConfig.image_L, self.image)
            time.sleep(0.2)
            rospy.loginfo("right image")
                
            R_data_len = len(RobotConfig.R_gesture_list)
            for i in np.arange(R_data_len):
                self.motion_service.setAngles(RobotConfig.joint_name, [RobotConfig.R_gesture_list[i][0], RobotConfig.R_gesture_list[i][1], RobotConfig.R_gesture_list[i][2], RobotConfig.R_gesture_list[i][3], RobotConfig.R_gesture_list[i][4], RobotConfig.R_gesture_list[i][5],
                                              RobotConfig.R_gesture_list[i][6], RobotConfig.R_gesture_list[i][7], RobotConfig.R_gesture_list[i][8], RobotConfig.R_gesture_list[i][9], RobotConfig.R_gesture_list[i][10],
                                              RobotConfig.R_gesture_list[i][11], RobotConfig.R_gesture_list[i][12], RobotConfig.R_gesture_list[i][13]], 0.3)                 
                time.sleep(0.025)

            time.sleep(0.5)
            # if self.image is not None and self.image.size != 0:
            cv2.imwrite(PathConfig.image_R, self.image)
            time.sleep(0.2)
            rospy.loginfo("left image")
                
            M_data_len = len(RobotConfig.M_gesture_list)
            for i in np.arange(M_data_len):
                self.motion_service.setAngles(RobotConfig.joint_name, [RobotConfig.M_gesture_list[i][0], RobotConfig.M_gesture_list[i][1], RobotConfig.M_gesture_list[i][2], RobotConfig.M_gesture_list[i][3], RobotConfig.M_gesture_list[i][4], RobotConfig.M_gesture_list[i][5],
                                              RobotConfig.M_gesture_list[i][6], RobotConfig.M_gesture_list[i][7], RobotConfig.M_gesture_list[i][8], RobotConfig.M_gesture_list[i][9], RobotConfig.M_gesture_list[i][10],
                                              RobotConfig.M_gesture_list[i][11], RobotConfig.M_gesture_list[i][12], RobotConfig.M_gesture_list[i][13]], 0.3)                    
                time.sleep(0.025)

            time.sleep(0.5)
            cv2.imwrite(PathConfig.image_M, self.image)
            time.sleep(0.2)
            rospy.loginfo("middle image")

            state = RobotConfig.autonomous_life_service.getState()
            if state == "disabled":
                RobotConfig.autonomous_life_service.setState("interactive")
            else:
                print("pepper is awake")

            self.look_around_flag ==False
            self.talk_start_pub.publish(self.start_talk_msg.data)
            rospy.loginfo("start_talk_msg is sent")

        else:
            print("The enviroment has been analyzed")


    def find_face(self):
            cv2.imwrite(PathConfig.face_img_path, self.camera) 

    def audio_paly(self):
        RobotConfig.audio_player_service.playFile(PathConfig.audio_file_path)


if __name__ == '__main__':
    rospy.init_node('robot_behavior', anonymous=True)
    robot_behavior = robot_behavior()
    robot_behavior.look_around()
    robot_behavior.find_face()
    rospy.spin()


