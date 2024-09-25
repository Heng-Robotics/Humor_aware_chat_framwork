#-*-coding:utf-8-*-

import csv
import random
from nltk.stem import PorterStemmer


class Joke_filter:
    def __init__(self):
        self.ps = PorterStemmer()

    # 读取CSV文件并查找包含特定关键词的行
    def find_jokes_by_keyword(self, csv_filename, keyword):

        rows_with_keyword = []
        jokes = []
        stemmed_keyword = self.ps.stem(keyword.lower())

        with open(csv_filename, newline='', encoding='utf-8') as csvfile:
            reader = csv.DictReader(csvfile)
            
            # 遍历CSV文件中的每一行
            for row in reader:
                # 获取关键词列，并进行小写和词干化处理
                keywords = [row['Keyword 1'].lower(), row['Keyword 2'].lower(), row['Keyword 3'].lower()]
                stemmed_keywords = [self.ps.stem(kw) for kw in keywords]
                
                # 检查词干化后的关键词是否包含目标关键词
                if stemmed_keyword in stemmed_keywords:
                    rows_with_keyword.append(row)
        
        # 按照Rating从大到小排序
        sorted_rows = sorted(rows_with_keyword, key=lambda x: float(x['Rating']), reverse=True)
        
        # 打印排序后的结果
        for row in sorted_rows:
            jokes.append(row['Joke'])
            matched_keyword = next(kw for kw in [row['Keyword 1'], row['Keyword 2'], row['Keyword 3']] if self.ps.stem(kw.lower()) == stemmed_keyword)
            # print(f"Joke: {row['Joke']}, Rating: {row['Rating']}, Matched Keyword: {matched_keyword}")

        if len(jokes) > 0:
            return jokes[0]
        else:
            return None


    def random_select_joke(self, joke_list):

        result = []
        valid_selection = None

        while valid_selection is None:
            for lst in joke_list:
                candidates = lst[:3]
                if candidates:  # 确保list不为空
                    selected = random.choice(candidates)
                    result.append(selected)
                    valid_selection = selected
                    break
            if valid_selection is None:  # 如果所有list都为空或没有有效选择，返回None
                break

        return valid_selection, result

        # return 


if __name__ == "__main__":

    csv_filename = 'dataset.csv' 
    keyword = 'sleep' 
    joke = ''
    
    joke_filter = Joke_filter()
    joke = joke_filter.find_jokes_by_keyword(csv_filename, keyword)
    print("joke: ", joke)
    
