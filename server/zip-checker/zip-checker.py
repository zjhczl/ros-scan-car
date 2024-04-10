# 压缩文件
# 监控文件变化
import os
import shutil
import time


def delete_old_zip_files(folder_path):
    # 获取当前时间戳
    current_time = time.time()
    # 遍历文件夹中的所有文件
    for file in os.listdir(folder_path):
        file_path = os.path.join(folder_path, file)
        # 检查文件是否为 ZIP 文件
        if file.endswith(".zip"):
            # 获取文件最后修改时间
            modified_time = os.path.getmtime(file_path)
            # 计算文件存在的天数
            days_diff = (current_time - modified_time) / (24 * 3600)
            # 如果文件存在的天数超过3天，则删除文件
            if days_diff > 3:
                os.remove(file_path)
                print("已删除 ZIP 文件:", file_path)


def compress_file(file_path, zip_path):
    shutil.make_archive(zip_path.replace(
        '.zip', ''), 'zip', os.path.dirname(file_path), os.path.basename(file_path))


# Set up the source and target directories
source_directory = '/home/zj/rosbag'
target_directory = '/home/zj/rosbag-zip'

# Create the target directory if it doesn't exist
if not os.path.exists(target_directory):
    os.makedirs(target_directory)


while True:
    # 查看已经存在的zip
    zips = os.listdir(target_directory)
    # Compress existing rosbags
    for file_name in os.listdir(source_directory):
        if file_name.endswith('.bag'):
            file_path = os.path.join(source_directory, file_name)
            zip_path = os.path.join(target_directory, file_name+".zip")
            if (os.path.basename(zip_path) not in zips):
                print("生成:"+zip_path)
                compress_file(file_path, zip_path)
            else:
                print("已存在:"+zip_path)

    delete_old_zip_files(target_directory)
    time.sleep(5)
