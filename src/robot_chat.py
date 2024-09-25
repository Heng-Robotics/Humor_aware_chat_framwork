#!/home/heng/Robot/py38/bin/python3.8
# -*- coding:utf-8 -*-
import json
import rospy
import time
# import pygame
from openai import OpenAI
import speech_recognition as sr
from joke import Joke_filter
from std_msgs.msg import String, Bool
from utils.config import PathConfig



class RobotChat:
    def __init__(self):
        
        self.joke_filter = Joke_filter()
        self.client = OpenAI(api_key = PathConfig.api_key)
        
        self.dialogue_pub = rospy.Publisher('/user_dialogue', String, queue_size=10)
        self.content_pub = rospy.Publisher('/speak_content', String, queue_size=1)
        self.beep_pub = rospy.Publisher('/beep_flag', Bool, queue_size=1)

        self.joke_sub = rospy.Subscriber('/topic_joke', String, self.joke_callback)
        self.emotion_sub = rospy.Subscriber('/emotion_state', String, self.emotion_callback)
        self.context_sub = rospy.Subscriber('/context_result', String, self.context_callback)
        self.start_talk_sub = rospy.Subscriber('/start_talk_flag', String, self.talk_callback)
        self.recognition_sub  = rospy.Subscriber('/recognition_flag', Bool, self.recognition_callback)

        self.recognizer = sr.Recognizer()
        self.language = "en-US"
        self.retries = 1

        self.history_conversation = []
        self.conversation_log = {}

        self.chat_flag  = True#是否继续进行chat
        self.beep_flag  = True
        self.beep_flag_msg  = Bool()
        self.dialogue_msg = String()#要发布出去的机器人的回应
        self.speak_content = String() #告诉机器人开始说话
        
        self.jokes_list = []
        self.context  = ""
        self.duration = 0.0
        self.user_question = ""
        self.instruction = ""
        self.emotion_state = "neutral"
        self.rob_response_len = 15
        self.file_path = PathConfig.audio_file_path
        self.rob_response = "Hello, I am social robot NAO. Nice to see you! Do you want to talk with me?"
        self.recog_erorr = "@error"
        self.bye = "It's been a pleasure chatting with you. I hope you have a great day. Bye!"
       
        rospy.loginfo("bot chat node initialized.")

    def emotion_callback(self, msg):
        try:
            if msg.data is not None:
                # rospy.loginfo("emotion state is received")
                self.emotion_state  = msg.data
        except Exception as e:
            rospy.logerr("There is an error in robot_chat/emotion_callback: ", str(e))

    def talk_callback(self, msg):
        try:
            if msg.data == "talk":
                # rospy.loginfo("start_talk_msg is received")
                self.speak_content.data  = self.rob_response
                self.content_pub.publish(self.speak_content.data)
            else:
                rospy.loginfo("waiting")
        except Exception as e:
            rospy.logerr("There is an error in robot_chat/talk_callbcak: ", str(e))

    def recognition_callback(self, msg):
        try:
            if msg.data == True:
                if self.rob_response_len > 45:
                    time.sleep(0.35*self.rob_response_len)
                elif self.rob_response_len >= 30 and self.rob_response_len <= 45:
                    time.sleep(0.335*self.rob_response_len)
                elif self.rob_response_len < 30 and self.rob_response_len >= 15:
                    time.sleep(0.32*self.rob_response_len)
                elif self.rob_response_len < 15:
                    time.sleep(0.27*self.rob_response_len)

                self.user_question = self.listen_and_recognize()
                rospy.loginfo("Recognizing result: %s ", self.user_question)

                if self.user_question.startswith("@error"):
                    prompt ="The user's speech was not recognized, possibly due to the volume being too low or because they spoke before the beep. You can tell the user in a cute tone that you didn't hear him clearly and the reason why you didn't hear him clearly. And ask if he can say it again."
                else:
                    prompt = self.sensor_gpt(self.context, self.emotion_state, self.user_question, self.history_conversation)
                    rospy.loginfo("prompt: %s", prompt)

                if prompt.startswith("@bye"):
                    self.content_pub.publish(self.bye)
                    self.save_conversation_log()
                    rospy.signal_shutdown("Interaction complete")
                else:                    
                    try:    
                        if prompt.startswith("@joke"):
                            if self.jokes_list[-1] is not None:
                                self.instruction = "please integrate the following joke to the reply, the content of this joke is: " + self.jokes_list[-1]
                                self.jokes_list.pop() 
                        else:
                            self.instruction = ""
                    except Exception as e:
                        rospy.logerr("There is an error in robot_chat/recognition_callback/joke selection: " + str(e))
                    self.rob_response = self.chat_gpt(prompt + self.instruction, self.user_question)
                    rospy.loginfo("Chat GPT response: %s", self.rob_response)
                    self.rob_response_len = len(self.rob_response.split())

                    self.speak_content.data  = self.rob_response
                    self.content_pub.publish(self.speak_content.data)#发布出去做tts
                    
                    self.history_conversation.append(self.user_question + ". " + self.rob_response)
                    self.dialogue_msg.data = self.rob_response + " " + self.user_question
                    self.dialogue_pub.publish(self.dialogue_msg)#发布出去做笑话筛选
                    # rospy.loginfo("Published dialogue: %s", self.dialogue_pub)
                    
                    round_number = len(self.conversation_log) + 1
                    self.conversation_log[round_number] = {
                        "user_question": self.user_question,
                        "prompt": prompt,
                        "rob_response": self.rob_response
                    }
        except Exception as e:
            rospy.logerr("There is an error in robot_chat/recognition_callback: ", str(e))


    def context_callback(self, msg):
        # rospy.loginfo("Received the context result")
        self.context = msg.data

    def joke_callback(self, msg):
        rospy.loginfo("Received joke: %s", msg.data)
        self.jokes_list.append(msg.data)

    def save_conversation_log(self):
        with open(PathConfig.conversation_log_dir, "w") as file:
            json.dump(self.conversation_log, file, indent=4)
        rospy.loginfo("Conversation log saved.")

    def listen_and_recognize(self):
        with sr.Microphone() as source:
            self.beep_pub.publish(self.beep_flag)
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Please say something:")
            audio = self.recognizer.listen(source)
            rospy.loginfo("Recognizing Now .... ")

            try:
                recognized_text = self.recognizer.recognize_google(audio, language=self.language)
                return recognized_text
            except sr.UnknownValueError:
                rospy.loginfo("Google Speech Recognition could not understand the audio.")
                return self.recog_erorr
            except sr.RequestError as e:
                rospy.loginfo("Could not request results from Google Speech Recognition service; {0}".format(e))
                return self.recog_erorr
            except Exception as e:
                rospy.loginfo("There is an error in robot_speech_recognition/listen_and_recognize: " + str(e))
            # time.sleep(0.5)
                return self.recog_erorr


    def sensor_gpt(self, user_question, emotion_state, context_info, history_conversation):
        prompt = "User question:" + str(user_question) + ". " + "Emotional state:  " + str(emotion_state)  + "Conetxt information:" + str(context_info) + ". " +  "History_conversation: " + str(history_conversation)
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[ 
                {"role": "system", "content": 
                """
                    You are a prompt generator. 
                    Your task is to analyse the given information to create a concise prompt within 80 words for a chatbot. 
                    The given information includes the user question, user's emotional state, context information, and the conversation history. 
                    You need to act as an assistant to tell the chatbot what you know and your analysis result about this user, so that chatbot can talk to the user with a more comprehensive understanding of the user.
                    Here are a few aspects you must keep in mind:
                    1. If the Context information is empty, do not generate or invent any context. Also, strictly base the prompt on the provided user question. Do not create your own! Never do that! Remember that there is always only one person in the current context. 
                    2. The prompt you generate should focus on the user's question and the key information from the latest chat history that are relevant to the current question, also taking into account the user's emotional state and situational information.
                    3. When you think the current situation requires humor (For example, the other person shows negative emotions such as sadness, unhappiness, etc.) , use @joke at the beginning of the generated prompt. But you can't just base on the user's emotional state, you also have to consider whether the current chat topic is appropriate for a joke or not. Don't propose a joke so frequently.
                    4. When the conversation should end (e.g., when the user wants to end the conversation and says goodbye), start the prompt with @bye.
                    For example:
                        "@joke The user is a 26-year-old Asian man sitting on a red couch in his living room. He seems to be at home, looks feeling stressful. He asked if you knew why it was so cold today. Anccording to the information from the conversation history, he mentioned that he has no money to bya heater. Please comfort him and tell him jokes to lighten up the mood."
                    Please generate the prompt in this style, but keep in mind that analyze and generate the prompt based on the specific information given to you.
                    Please output only the generated prompt, thanks!
         
             """},
                 {"role": "assistant", "content": prompt}
            ],
            temperature=0.5,
            n=1,
            max_tokens=80
        )
        return str(response.choices[0].message.content)

    def chat_gpt(self, sensor_prompt, user_question):
        prompt = "User question or response is:" + str(user_question) + ". " + "The instruction analyzed by the chat assistant is: " + sensor_prompt
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": 
                 """
                You are a robot named NAO. 
                Please response to the user in a natural and relax way according to the user's question. 
                In addition, you can refer to the information sent by the chat assistant about the user, which allows you to understand the user better. But remember that, just refer to these information when you think it's necessary, otherwise, you just need to foucs on the user's question.
                You need to use simple words to response the user by using within 60 words. 
                Keep your responses flexible: sometimes use brief, casual sentences for small talk. 
                Please avoid using itemized lists or bullet points to present information.
                Please do not include double and single quotes in the generated content.
                Do not alway use "Hey, there" as the begining of each talk.

                 """ },
                 {"role": "user", "content": prompt}
            ],
            temperature=0.5,
            n=1,
            max_tokens=80
        )
        return str(response.choices[0].message.content)

if __name__ == '__main__':
    rospy.init_node('robot_chat', anonymous=True)
    try:
        robot_chat = RobotChat()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
