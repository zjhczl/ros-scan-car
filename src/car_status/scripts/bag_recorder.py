#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import subprocess
import os
import re
import time
import threading


class BagRecorder:
    def __init__(self):
        self.bag_process = None
        self.is_recording = False
        self.numofthreading = 0
        self.timer = 0

        # 初始化ROS节点
        rospy.init_node('bag_recorder', anonymous=True)
        # 订阅/car_status话题
        rospy.Subscriber('/car_status', String, self.callback)
        rospy.spin()

    def cleanup_directory(self, directory_path):
        # 获取目录中所有文件的列表
        files = [f for f in os.listdir(directory_path) if os.path.isfile(
            os.path.join(directory_path, f))]

        # 检查文件数量是否超过30个
        if len(files) > 20:
            # 按文件名中的日期和时间排序文件
            files.sort(key=lambda x: re.findall(
                r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', x))

            # 计算需要删除的文件数量
            num_files_to_delete = len(files) - 20

            # 删除最早的文件
            for i in range(num_files_to_delete):
                file_to_delete = os.path.join(directory_path, files[i])
                os.remove(file_to_delete)
                print(f"Deleted file: {file_to_delete}")

    def time_checker(self):
        if (self.numofthreading == 0):
            self.numofthreading = 1
            while True:
                time.sleep(1)
                self.timer = self.timer - 1
                if (self.timer < 0):
                    if (self.is_recording):
                        self.stop_recording()
                    self.numofthreading = 0
                    break
        else:
            print("已有线程启动...")
            pass

    def callback(self, msg):
        # 检查接收到的消息，并根据消息内容控制bag录制
        if msg.data == 'start':
            self.cleanup_directory('/ssd/rosbag')

            if (self.is_recording):
                self.stop_recording()
                time.sleep(1)
                self.start_recording()
            else:
                self.start_recording()

            self.timer = 60
            t = threading.Thread(target=self.time_checker)
            t.start()

        elif msg.data == 'stop' and self.is_recording:
            # 停止录制bag文件
            self.stop_recording()

    def start_recording(self):
        # 定义要录制的话题列表
        # topics_to_record = ['/topic1', '/topic2']  # 根据需要录制的话题修改这里
        # 开始录制bag文件
        command = 'rosbag record -a -o /ssd/rosbag/ros-all'
        # print(command)
        self.bag_process = subprocess.Popen(
            command, shell=True, preexec_fn=os.setsid)
        self.is_recording = True
        print("Started recording bag file.")
        rospy.loginfo("Started recording bag file.")

    def stop_recording(self):
        # 结束录制bag文件
        os.killpg(os.getpgid(self.bag_process.pid), subprocess.signal.SIGINT)
        self.bag_process = None
        self.is_recording = False
        print("Stopped recording bag file.")
        rospy.loginfo("Stopped recording bag file.")


if __name__ == '__main__':
    try:
        BagRecorder()
    except rospy.ROSInterruptException:
        pass
