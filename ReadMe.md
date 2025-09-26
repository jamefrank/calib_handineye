# 简介
标定机械臂+相机的手眼标定工具  hand in eye

# 编译
```
sudo apt update

sudo apt install -y \
    libyaml-cpp-dev \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libceres-dev \
    libspdlog-dev \
    libboost-filesystem-dev \
    libboost-system-dev

mkdir build && cd build
cmake ..
make 
```

# 命令行说明
```
usage: ./calib_eyeinhand --input=string --output=string [options] ... 
options:
  -i, --input           Input directory containing images + arm_pose.txt + config.yaml (string)
  -e, --extension       File extension of images (string [=.png])
  -o, --output          Output directory containing calib results (string)
  -t, --board-type      board type (string [=charuco])
  -w, --board-width     Number of inner corners on the chessboard pattern in x direction (int [=3])
  -h, --board-height    Number of inner corners on the chessboard pattern in y direction (int [=2])
  -s, --square-size     Size of one square in mm (double [=36])
  -m, --marker-size     Size of one square in mm (double [=27])
      --verbose         verbose when calib
  -?, --help            print this message
```

# 数据采集
使用采集工具多角度采集数据
采集要求:
1. 标定板不动
2. 机械臂底座不动
3. 采集数据时机械臂不允许晃动,必须完全静止才能采集,防止微小抖动
4. 采集时图像需要看清楚所有角点
5. 采集尽量多角度采集数据,保证6个自由度都有变化,至少采集12张图片分别对应 6个自由度各自变换两次
6. 采集图像需要清晰不模糊

# 数据预处理
```
# 1. 修改scripts/ornize_data.py中的输入文件夹和输出文件夹参数,整理采集数据为标定程序可以识别的格式
#source_dir = '/home/frank/Downloads/scene_biaoding'  # 替换为你的源文件夹路径
#target_dir = '/home/frank/Pictures/handeye/data_0906_2'  # 替换为你想保存处理后文件的目标文件夹路径

# 2. 运行脚本,数据存放在了data_0906_2文件夹中
python scripts/orgnize_data.py

# 3.在data_0906_2文件夹中创建config.yaml文件,修改其中的相机内参为对应相机的内参,示例参照data文件夹
```

# 运行
```
cd build
./calib_eyeinhand -i ../data_0906_2 -o ../output_0906_2
```

# 标定结果
```
output_0906_2/calibration.txt
```

# 点云可视化验证
1. 使用采集程序采集验证数据,例如多角度采集固定好的零件,此时基座也不允许移动
2. 修改valid_cloud.py脚本参数
```
T_cam2gripper = np.array(
    [
        [-0.9920709027701955, 0.01558599439399975, 0.1247092645134259, -0.02204443318338138],
        [-0.01353494389720342, -0.9997591243138533, 0.01727711332650279, 0.06733309933869475],
        [0.1249485060752187, 0.01545218851643587, 0.9920428925704894, -0.0753261971755169],
        [0.0, 0.0, 0.0, 1.0]       
    ]
)
input_dir = "/home/frank/Downloads/scene_001"
output_dir = "/home/frank/Pictures/handeye/output_cloud"
```
3. 输出结果在output_cloud
4. 打开cloudcompare软件,拖入output_cloud内的所有点云,观察是否重合良好
