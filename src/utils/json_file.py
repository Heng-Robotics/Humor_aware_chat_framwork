#-*-coding:utf-8-*-
import json
import os

def json_write(data, json_file_path):
    with open(json_file_path, 'w', encoding='utf-8') as json_file:
        json.dump(data, json_file, ensure_ascii=False, indent=4)
    json_file.close()
    # print('data has been updated')


def json_read(json_file_path):
    with open(json_file_path, 'r', encoding='utf-8') as json_file:
        data = json.load(json_file)
    if not isinstance(data, str):
        raise ValueError("JSON data is not string")
    
    return data

def json_update_face_info(name1, name2, name3, data1, data2, data3, json_file_path):
    with open(json_file_path, 'r') as json_file:
        data = json.load(json_file)

    data[name1] = data1
    data[name2] = data2
    data[name3] = data3

    json_file.close()
    json_write(data, json_file_path)


def json_update_context(name, new_data, json_file_path):
    with open(json_file_path, 'r') as json_file:
        data = json.load(json_file)

    data[name] = new_data
    json_file.close()
    json_write(data, json_file_path)

def json_create_info(json_file_path):
    initial_data = {
        'The user age is ': '',
        'gender is ': '',
        'race is': ''
    }

    with open(json_file_path, 'w') as json_file:
        json.dump(initial_data, json_file)

    json_file.close()
    print('Context json file has been created')

def json_create_context(json_file_path):
    initial_data = {
        'context': ''
    }

    # json_file_path = Config.json_path
    with open(json_file_path, 'w') as json_file:
        json.dump(initial_data, json_file)

    json_file.close()
    print('Info json file has been created')