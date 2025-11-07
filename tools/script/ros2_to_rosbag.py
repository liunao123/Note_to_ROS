#!/usr/bin/env python3

import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def discover_topics_in_bag(bag_path):
    """
    打开一个 ROS2 Bag 文件并发现其中记录的所有话题及其类型。
    
    参数:
        bag_path (str): Bag 文件夹的路径
    """
    
    # 1. 设置存储和转换选项
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    
    # 2. 创建读取器并打开Bag文件
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # 3. 获取所有话题的元数据
    topic_types = reader.get_all_topics_and_types()
    
    # 4. 创建一个字典来存储话题信息
    topic_info = {}
    # for topic_type in topic_types:
    #     topic_name = topic_type.name
    #     message_type = topic_type.type
    #     topic_info[topic_name] = message_type
    #     print(f"Found Topic: {topic_name} | Type: {message_type}")
    
    # 可选：如果你想进一步处理消息，可以按话题遍历
    # 但请注意，这通常会按时间顺序读取所有消息，而不是按话题
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        # 反序列化消息
        # msg_type = get_message(topic_info[topic])
        # msg = deserialize_message(data, msg_type)
        # print(f"Topic: {topic}, Time: {t}, Message: {msg}")
        print(f"Topic: {topic}, Time: {t} ")
    
    reader.close()
    return topic_info

def main():
    # 初始化 ROS2 Python 客户端库
    rclpy.init()
    
    # 替换成你的 Bag 文件路径
    bag_path = '/home/tyjt/Desktop/by_gnss_ws/data/by_0908/by_0908.db3'
    
    try:
        topics = discover_topics_in_bag(bag_path)
        print(f"\nDiscovered {len(topics)} unique topics in the bag.")
    except Exception as e:
        print(f"Failed to read bag file: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()