#!/home/heng/Robot/py38/bin/python3.8
#-*-coding:utf-8-*-
"""
这个节点从captioning节点订阅context分析完成的信号(analysis_flag)
然后对captioning和deepface的u处理信息进行整合,得到context_result并发布给Robot_chat节点
"""

import rospy
from openai import OpenAI
from utils.json_file import json_read
from std_msgs.msg import String
from utils.config import PathConfig

class GPT_Assistant():

    def __init__(self):
        self.context_analysis_result = ''
        self.client = OpenAI(api_key = PathConfig.api_key)
        self.analysis_sub = rospy.Subscriber("/analysis_flag", String, self.callback)
        self.conetxt_pub = rospy.Publisher('/context_result', String, queue_size=1)

        self.conext_msg = String()#要发布出去的context内容

    def context_info_analysis(self):    
        face_info = json_read(PathConfig.json_path_face)
        caption_info = json_read(PathConfig.json_path_captioning)

        self.context_analysis = self.client.completions.create(
          model="gpt-3.5-turbo-instruct",
          prompt = 
          f"""
          Below are two sets of information. Please generate a descriptive sentence based on these details.
          The first set of information is about a person and includes the following attributes: {face_info}
          The second set of information is about the person's surrounding environment: {caption_info}
          Remember that, if the presence of a person is detected in images from multiple viewpoints, it is must be the same person.
          Prioritize describing the viewpoint that includes the person first, and then integrate details from the other images.
          Please generate a comprehensive description according to above information within 70 words. 
          Here is an generated example:
          The generated description is: He is an Asian man, around 20 years old. He is seen wearing glasses and sitting in front of a desk. To his left, there is a computer monitor on the desk. To his right, there is a filing cabinet. This environment suggests that he is likely in an office or a study, working or studying.
          """,
        max_tokens=100,
        temperature=0.5,
        )
        return self.context_analysis.choices[0].text.strip()
    
    def callback(self,msg):
        try:
            if msg.data == 'collection_finish':
                self.context_analysis_result = self.context_info_analysis()
                # rospy.loginfo("#recevied the context analysis")
                self.conext_msg.data = self.context_analysis_result
                self.conetxt_pub.publish(self.conext_msg.data)

                print(self.context_analysis_result)
                rospy.loginfo("Context analysis done!!!")
            else:
                rospy.loginfo("Analysing context...")
        except:
            print("There is an error in gpt_assistant")  


if __name__ == '__main__':
    rospy.init_node('gpt_assistant', anonymous=True)
    gpt_assistant = GPT_Assistant()
    rospy.spin()