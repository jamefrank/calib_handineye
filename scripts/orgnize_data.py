import os
import shutil
import json
import math

# 源文件夹路径和目标文件夹路径
source_dir = '/home/frank/Downloads/scene_biaoding'  # 替换为你的源文件夹路径
target_dir = '/home/frank/Pictures/handeye/data_0919'  # 替换为你想保存处理后文件的目标文件夹路径
txt_path = os.path.join(target_dir, 'arm_pose.txt')

# 确保目标文件夹存在
if not os.path.exists(target_dir):
    os.makedirs(target_dir)

# === 第一部分：复制 image_2d.png（保持不变）===
for root, dirs, files in os.walk(source_dir):
    timestamp = os.path.basename(os.path.dirname(root))
    for file in files:
        if file == 'image_2d.png':
            new_filename = f"{timestamp}.png"
            source_file = os.path.join(root, file)
            target_file = os.path.join(target_dir, new_filename)
            shutil.copyfile(source_file, target_file)
            print(f"Copied and renamed {source_file} to {target_file}")
            
        if file == 'point_cloud.ply':
            new_filename = f"{timestamp}.ply"
            source_file = os.path.join(root, file)
            target_file = os.path.join(target_dir, new_filename)
            shutil.copyfile(source_file, target_file)
            print(f"Copied and renamed {source_file} to {target_file}")
            
# === 第二部分：读取 tcp_pose.json 并按时间戳排序写入 ===
json_files = []

# 收集所有 tcp_pose.json 文件及其时间戳
for root, dirs, files in os.walk(source_dir):
    for file in files:
        if file == 'robot_status.json':
            json_path = os.path.join(root, file)
            folder_name = os.path.basename(root)  # 即 20250905_035504
            timestamp = folder_name
            json_files.append(json_path)

# 按时间戳排序（字符串排序即可，因为格式统一为 YYYYMMDD_HHMMSS）
json_files.sort()
for json_file_path in json_files:
    print(f"处理文件: {json_file_path}")

# 写入 txt 文件，保证顺序
with open(txt_path, 'w') as output_file:
    for json_file_path in json_files:
        try:
            with open(json_file_path, 'r') as f:
                data = json.load(f)
                robot_status = data.get('robot_status', {})
                tcp_pose = robot_status.get('tcp_pose', [])
                if len(tcp_pose) == 6:
                    converted = [
                        tcp_pose[0] * 1000,           # x (m → mm)
                        tcp_pose[1] * 1000,           # y (m → mm)
                        tcp_pose[2] * 1000,           # z (m → mm)
                        math.degrees(tcp_pose[3]),    # rx (rad → deg)
                        math.degrees(tcp_pose[4]),    # ry (rad → deg)
                        math.degrees(tcp_pose[5])     # rz (rad → deg)
                    ]
                    line = ' '.join(f"{val:10.6f}" for val in converted)
                    # line = json_file_path + " : " + line
                    output_file.write(line + '\n')
                else:
                    print(f"警告: {json_file_path} 中 tcp_pose 数据长度不为6")
        except Exception as e:
            print(f"读取失败: {json_file_path}, 错误: {e}")

print("操作完成")