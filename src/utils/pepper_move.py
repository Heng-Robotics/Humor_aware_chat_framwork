#!/usr/bin/env python
import sys
import qi
import rospy
import time
import threading
import cv2
import os 
from utils.config import Config
from pepper_config import Pepper_config
from pepper_control.msg import Topic_msg
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
sys.path.remove('/home/heng/catkin_ws/devel/lib/python3/dist-packages')
from cv_bridge import CvBridge, CvBridgeError
# sys.path.append('/home/heng/catkin_ws/devel/lib/python3/dist-packages')

from naoqi import ALProxy


class Pepper_move():
    def __init__(self):
        # self.topic_data = Topic_msg()
        self.bridge = CvBridge()
        self.context_flag = String()
        self.rotation_angle_sub = rospy.Subscriber('/robot_motion', Topic_msg, self.rotation_angle_callback)
        # self.move_again_sub = rospy.Subscriber('/move_again', Bool, self.move_again_callback)
        self.image_sub = rospy.Subscriber('/image_view/image_raw', Image, self.image_callback)
        self.context_flag_pub = rospy.Publisher("/context", String, queue_size=1)

        self.rot_angle_x = 0.0
        self.rot_angle_y = 0.0
        self.face_ratio = 0.0
        self.is_person_detected = False
        self.face_ratio_threshold = 0.15  # Adjust this threshold for a suitable face size
        self.move_again_threshold = 5 

        self.motion_service = Pepper_config.motion_service
        # self.motion_service.setAngles("HeadPitch", 0, 0.1)
        self.posture_service = Pepper_config.posture_service
        self.posture_service.goToPosture("StandInit", 1.0)
        self.detection_service = Pepper_config.detection_service

        self.context_flag.data = 'no_caption'
        self.look_around_flag = True
        self.move_head_flag = True
        self.move_step = 0.1
        self.frequency = 1.0
        # self.script_directory = os.path.dirname(os.path.abspath(__file__))

        self.tracker = ALProxy("ALTracker", "192.168.0.198", 9559)
        self.tracker.registerTarget("Face", 0.1)


    def rotation_angle_callback(self, msg):
        if len(msg.float_list) == 3:
            self.rot_angle_x = msg.float_list[0]
            self.rot_angle_y = msg.float_list[1]
            self.face_ratio     =  msg.float_list[2]
            # print("self.rot_angle_x", self.rot_angle_x)
            # print("self.rot_angle_y", self.rot_angle_y)
            self.context_flag_pub.publish(self.context_flag)


    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # cv2.imshow("face_image", self.image)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def look_around(self):

        if self.look_around_flag ==True:
            #head turn right and save an image
            self.motion_service.setAngles("HeadYaw", -0.8, 0.15)
            time.sleep(2)
            cv2.imwrite(Config.image_L, self.image)

            #head turn left and save an image
            self.motion_service.setAngles("HeadYaw", 0.8, 0.15)
            time.sleep(3)
            cv2.imwrite(Config.image_R, self.image)

            #return to initial position
            self.motion_service.setAngles("HeadYaw", 0, 0.15)
            time.sleep(2)   
            cv2.imwrite(Config.image_M, self.image)

            self.look_around_flag ==False

            self.context_flag.data = 'caption'
            
        else:
            print("The enviroment has been analyzed")

    def track_human_x(self):
        # while self.rot_angle_x != 0 and self.move_flag == True:
        if self.rot_angle_x != 0:
            if self.rot_angle_x < 0:
                self.move_base_rotation(-0.5)
                # time.sleep(0.1)
            else:
                self.move_base_rotation(0.5)
                # time.sleep(0.1)
        else:
            self.stop_move()
            time.sleep(0.1)

    def track_human_y(self):
        while True:
            rot_before = self.rot_angle_y

            if abs(self.rot_angle_y) > 0.1 and self.move_head_flag == True:
                self.move_head(self.rot_angle_y)
                # time.sleep(1)
                self.move_head_flag = False        

            rot_after = self.rot_angle_y
            if abs((rot_after - rot_before)> 0.01) and abs(self.rot_angle_y) > 0.1:
                self.move_head_flag = True
                print("true again")

    def face_tracking(self):
        # while True:
            # self.detection_service.enableTracking(True)
            # self.motion_service.setAngles("HeadYaw", 0, 0.1)
            self.tracker.track("Face")
            # time.sleep(0.05)
         
            
    def move_base_toward(self, x):
        self.motion_service.moveToward(x, 0.0, 0.0, [["Frequency", self.frequency]])

    def move_base_rotation(self, angle):
        self.motion_service.move(0.0, 0.0, angle)
        time.sleep(0.1)

    def move_head(self, angle):

        current_angle = self.motion_service.getAngles("HeadPitch", False)
        next_angle = current_angle[0] + angle
        if next_angle < -0.4:
            next_angle = -0.4
        elif next_angle > 0.0:
            next_angle = 0.0

        self.motion_service.setAngles("HeadPitch", next_angle, 0.1)
        time.sleep(0.1)
        return

        # currentAngle = self.motion_service.getAngles("HeadPitch", True)[0]
        # if headypos > (height/2 - 5):
        #     pdiff = abs((height/2 - 5) - headypos)
        #     turn = radians(pdiff / dheight)
        #     speed = calculate_speed(turn)
        #     if currentAngle < 0.51:
        #         self.motion_service.setAngles("HeadPitch", currentAngle + turn, speed)

        # # MOVE HEAD DOWN
        # if headypos < (height/2 + 5):
        #     pdiff = abs((height/2 - 5) - headypos)
        #     turn = radians(pdiff / dheight)
        #     speed = calculate_speed(turn)
        #     if currentAngle > -0.65:
        #         self.motion_service.setAngles("HeadPitch", currentAngle - turn, speed)
            

            # if self.face_ratio <= self.face_ratio_threshold:
            #     # print("self.face_ratio <= self.face_ratio_threshold", self.face_ratio)
            #     self.move_base_rotation(0.5)

            #     print("base_move_flag")
            # else:
            #     self.move_head(self.rot_angle_x)
            #     print("head_move_flag")


            # while True:
            #     self.move_base_toward(self.move_step)
            #     if self.face_ratio > self.face_ratio_threshold:
        # self.move_flag = True

    # def move_again_callback(self, flag):
    #     if flag.data == 'move_again':
    #         self.move_flag = False
    #         self.move_base_rotation(self.rot_angle_x)
    #     else:
    #         self.move_flag = True

    #     # Determine if a person is detected based on face detection result
    #     self.is_person_detected = True  # Set this variable based on your face detection result


    def stop_move(self):
        self.motion_service.stopMove()


if __name__ == '__main__':

    # ip = "192.168.0.198"
    # port = 9559
    # session = qi.Session()
    # try:
    #     session.connect("tcp://" + ip + ":" + str(port))
    # except RuntimeError:
    #     print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
    #            "Please check your script arguments. Run with -h option for help.")
    #     sys.exit(1)

    rospy.init_node('pepper_move', anonymous=True)

    pepper_move = Pepper_move()
    pepper_move.look_around()
    
    # my_thread = threading.Thread(target=pepper_move.face_tracking)
    # my_thread.start()
    pepper_move.face_tracking()

    while not rospy.is_shutdown():   
        pepper_move.track_human_x()

    pepper_move.tracker.stopTracker()

    # rospy.spin()

