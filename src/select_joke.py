#!/home/heng/Robot/py38/bin/python3.8
#-*-coding:utf-8-*-

import rospy
from std_msgs.msg import String
import csv
from openai import OpenAI
from nltk.stem import PorterStemmer
from utils.config import PathConfig

class JokeFilterNode:
    def __init__(self):
        self.ps = PorterStemmer()
        self.client = OpenAI(api_key = PathConfig.api_key)
        rospy.init_node('joke_teller_node', anonymous=True)
        
        self.joke_pub = rospy.Publisher('/topic_joke', String, queue_size=10)
        self.dialogue_sub = rospy.Subscriber('/user_dialogue', String, self.dialogue_callback)

        self.csv_filename = rospy.get_param('~csv_filename', PathConfig.joke_path)
        rospy.loginfo("Joke filter node initialized.")

        self.keyword = ''
        self.keyword_list = []

    def extract_keywords(self, user_dialogue):
        response = self.client.completions.create(
            model="gpt-3.5-turbo-instruct",
            prompt= f"""Extract the most important keyword from the following text. 
                                        Here is a list of previously extracted keywords: {', '.join(self.keyword_list)}. Please never selecting any keywords that have already in this list.
                                        Please extract only a single word instead of a phrase. Please try to choose nouns or verbs from the dialogue: {user_dialogue}/
                                        Please output only the extracted keyword.
                                        """,
            max_tokens=10,
            temperature=0.5,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0
        )
        print(f"""previously extracted keywords: {', '.join(self.keyword_list)}.""")
        keyword = response.choices[0].text.strip()
        return keyword

    def find_jokes_by_keyword(self, keyword):
        rows_with_keyword = []
        jokes = []
        stemmed_keyword = self.ps.stem(keyword.lower())

        with open(self.csv_filename, newline='', encoding='utf-8') as csvfile:
            reader = csv.DictReader(csvfile)

            for row in reader:
                # 获取关键词列，并进行小写和词干化处理
                keywords = [row['Keyword 1'].lower(), row['Keyword 2'].lower(), row['Keyword 3'].lower()]
                stemmed_keywords = [self.ps.stem(kw) for kw in keywords]
                
                # 检查词干化后的关键词是否包含目标关键词
                if stemmed_keyword in stemmed_keywords:
                    rows_with_keyword.append(row)

        sorted_rows = sorted(rows_with_keyword, key=lambda x: float(x['Rating']), reverse=True)
        
        # 打印排序后的结果
        for row in sorted_rows:
            jokes.append(row['Joke'])

        if len(jokes) > 0:
            return jokes[0]
        else:
            return None

    def dialogue_callback(self, msg):
        # rospy.loginfo("Received user dialogue: %s", msg.data)
        self.keyword = self.extract_keywords(msg.data)
        self.keyword_list.append(self.keyword)
        joke = self.find_jokes_by_keyword(self.keyword)

        if joke:
            joke_msg = String()
            joke_msg.data = joke
            self.joke_pub.publish(joke_msg)
            rospy.loginfo("The keyword from the dialogue is: %s", self.keyword)
            rospy.loginfo("Published joke: %s", joke)
        else:
            rospy.loginfo("No joke found for the keyword: %s", self.keyword)

    def run(self):
        try:
            rospy.spin()
        
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down the joke node.")

if __name__ == '__main__':
    try:
        node = JokeFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
