#!/home/heng/Robot/py38/bin/python3.8
#-*-coding:utf-8-*-
"""
该节点负责进行captioning和deepface的人脸信息分析;
对于captioning,只进行一次信息采集和处理;
对于deepface,则是先判断图像中的人脸是否清晰.
如果不清晰,则向robot_behaviorie节点发送再次采集人脸照片的请求;
如果人脸清晰,则向GPT-assistant节点发送已经完成context信息采集,可以开始contextn分析的信号.
"""

import gc
import os
import time
import rospy
import torch
from collections import Counter
from PIL import Image
from std_msgs.msg import String, Bool
from deepface import DeepFace
from lavis.models import load_model_and_preprocess
from utils.config import PathConfig
from utils.json_file import json_write

class Context_info():
        
    def __init__(self):
        rospy.init_node('context_info', anonymous=True)
        self.captions = []
        self.info = {}
        self.context_analysis = ""
        self.face_info_description = ""
        self.caption_info_description = ""
        self.caption_list = ["It's a description of the person's surroundings as seen from the right,  left, and middle viewpoints respectively"]

        self.face_found_flag = String()
        self.face_found_flag.data = 'face_not_found'
        self.context_collect_finish_flag = String()
        self.context_collect_finish_flag.data = 'collection_finish'       
        self.emotion_state_msg = String()
        self.emotion_state_msg.data = ''
        self.emotion_flag_sub= rospy.Subscriber("/stop_picture_flag", Bool, self.emotion_callback)
        self.context_sub= rospy.Subscriber("/context", String, self.callback)
        self.deepface_pub = rospy.Publisher("/deepface_flag", String, queue_size=1)
        self.context_pub = rospy.Publisher("/analysis_flag", String, queue_size=1)
        self.emotion_pub = rospy.Publisher("/emotion_state", String, queue_size=1)

        self.face_imgs = [os.path.join(PathConfig.face_img_folder, file_name) for file_name in os.listdir(PathConfig.face_img_folder) if file_name.endswith(('.jpg', '.jpeg', '.png'))]
        self.captioning_imgs = [os.path.join(PathConfig.captioning_img_folder, file_name) for file_name in os.listdir(PathConfig.captioning_img_folder) if file_name.endswith(('.jpg', '.jpeg', '.png'))]
        self.model, self.vis_processors, _ = load_model_and_preprocess(name="blip_caption", model_type="base_coco", is_eval=True, device=torch.device( "cpu"))

    def emotion_callback(self, msg):
        try:
            if msg.data:
                self.deepface_emotion()
            else:
                rospy.loginfo("emotion is proecssed!")
        except:
            print("There is an error in captioning/emotion_callback")

    def callback(self, msg):
        try:
            if msg.data == 'new_picture_coming':
                self.deepface_info()
            else:
                rospy.loginfo("face is found!")
        except:
            print("There is an error in captioning/callback")

    #---------processing caption image----------------
    def generate_caption(self):
        while len(self.captioning_imgs) < 3:
            self.captioning_imgs = [os.path.join(PathConfig.captioning_img_folder, file_name) for file_name in os.listdir(PathConfig.captioning_img_folder) if file_name.endswith(('.jpg', '.jpeg', '.png'))]
            time.sleep(0.5)
            print("Loading captioning images")
        
        for image_path in self.captioning_imgs:
            try:
                raw_image = Image.open(image_path).convert("RGB") 
                image = self.vis_processors["eval"](raw_image).unsqueeze(0).to(device=torch.device( "cpu"))
                caption = self.model.generate({"image": image})
                caption_description = os.path.splitext(os.path.basename(image_path))[0] + " image shows that "  + caption[0]

                self.caption_list.append(caption_description)
                self.caption_info_description = ". ".join(self.caption_list) 

            except Exception as e:
                print(f"Error processing image {image_path}: {e}")

        json_write(self.caption_info_description, PathConfig.json_path_captioning)
        rospy.loginfo("captioning done!!!")

    #------processing face image-------------
    def process_result(self, result):
        # information
        dominant_gender = result['gender']
        age = result['age']
        dominant_race = result['dominant_race']
        dominant_emotion = result['dominant_emotion']
        
        # describe the information according to different situations
        if dominant_gender.lower() == 'woman':
            gender_pronoun = 'She'
            possessive_pronoun = 'Her'
        else:  
            gender_pronoun = 'He'
            possessive_pronoun = 'His'

        description = "{} is a {}. {} age is around {}. {} dominant race is {}.".format(
            gender_pronoun, dominant_gender.lower(), possessive_pronoun, age, possessive_pronoun, dominant_race.lower())
        # description = "{} is a {}. {} age is around {}. {} dominant race is {}. {} current emotional state is {}.".format(
        #     gender_pronoun, dominant_gender.lower(), possessive_pronoun, age, possessive_pronoun, dominant_race.lower(), possessive_pronoun, dominant_emotion.lower())

        return description

    def deepface_info(self):
        while len(self.face_imgs) < 1:
            self.face_imgs = [os.path.join(PathConfig.face_img_folder, file_name) for file_name in os.listdir(PathConfig.face_img_folder) if file_name.endswith(('.jpg', '.jpeg', '.png'))]
            time.sleep(0.5)
            # print("Loading face images")

        for face_image in self.face_imgs:
            # print("Processing face images")
            try:
                self.info = DeepFace.analyze(face_image, ['age', 'gender', 'race', 'emotion'])
                rospy.loginfo("deepface done!!!")

                if self.info is not None:
                    self.face_info_description = self.process_result(self.info)
                    json_write(self.face_info_description, PathConfig.json_path_face)
                    self.context_pub.publish(self.context_collect_finish_flag)
                    # rospy.loginfo("### context info collection finish!!")
            except Exception as e:
                # if no result, asking for another photo
                self.deepface_pub.publish(self.face_found_flag)
                print(f"there is no face found, the error is {e}")
                continue

        torch.cuda.empty_cache()
        gc.collect()

    def deepface_emotion(self):
        
        if not os.path.exists(PathConfig.image_dir):
            rospy.logerr(f"Image directory '{PathConfig.image_dir}' does not exist.")
            return

        emotions_list = []  # 存储所有图片的情绪结果

        # 遍历文件夹中的所有图片
        for image_name in os.listdir(PathConfig.image_dir):
            image_path = os.path.join(PathConfig.image_dir, image_name)

            try:
                # 使用 DeepFace 分析图片的情绪
                analysis = DeepFace.analyze(img_path=image_path, actions=['emotion'])
                emotion = analysis['dominant_emotion']
                emotions_list.append(emotion)
                # rospy.loginfo(f"Processed {image_name}, detected emotion: {emotion}")
            except Exception as e:
                rospy.logerr(f"Error analyzing {image_name}: {str(e)}")
                continue

        if emotions_list:
            emotion_count = Counter(emotions_list)
            most_common_emotion = emotion_count.most_common(1)[0][0]
            self.emotion_state_msg.data = most_common_emotion
            self.emotion_pub.publish(self.emotion_state_msg.data)
            # rospy.loginfo(f"@#@#@##@#@#The most common emotion is: {most_common_emotion}")
        else:
            rospy.loginfo("No emotions detected in any images.")


if __name__=='__main__':
    context = Context_info()
    context.generate_caption()
    context.deepface_info()

    rospy.spin()
