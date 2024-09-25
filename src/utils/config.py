# coding:utf-8

class PathConfig():
#face recognition model
   face_model = '/home/heng/catkin_ws/src/pepper_adaptive_humor/model/face_detection_yunet_2022mar.onnx'

   #caption images
   image_L = '/home/heng/catkin_ws/src/pepper_adaptive_humor/image/context/left.jpg'
   image_R= '/home/heng/catkin_ws/src/pepper_adaptive_humor/image/context/right.jpg'
   image_M = '/home/heng/catkin_ws/src/pepper_adaptive_humor/image/context/middle.jpg'
   captioning_img_folder = "/home/heng/catkin_ws/src/pepper_adaptive_humor/image/context"

   #encoding and deep face 
   face_img_path = '/home/heng/catkin_ws/src/pepper_adaptive_humor/image/face/face.jpg'
   face_img_folder = "/home/heng/catkin_ws/src/pepper_adaptive_humor/image/face/"

   #json
   json_path_captioning = '/home/heng/catkin_ws/src/pepper_adaptive_humor/scripts/context.json'
   json_path_face = '/home/heng/catkin_ws/src/pepper_adaptive_humor/scripts/info.json'
   json_folder_path = '/home/heng/catkin_ws/src/pepper_adaptive_humor/scripts'

   #audio
   audio_file_path = '/home/heng/catkin_ws/src/pepper_adaptive_humor/files/robot_speech.wav'
   joke_path = '/home/heng/catkin_ws/src/pepper_adaptive_humor/files/dataset.csv'

   api_key='sk-proj-number'
   
   #emotion
   image_dir = "/home/heng/catkin_ws/src/pepper_adaptive_humor/image/emotion/"

   conversation_log_dir = '/home/heng/catkin_ws/src/pepper_adaptive_humor/files/25_2.txt'