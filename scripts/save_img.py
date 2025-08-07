#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

def main():
    rospy.init_node('save_single_image', anonymous=True)
    
    # 获取当前时间并格式化为字符串
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"image_{timestamp}.png"

    print(f"等待接收一帧图像... 将保存为 {filename}")
    
    # 等待一次 /camera/color/image_raw 消息
    msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=5.0)

    bridge = CvBridge()
    try:
        # 转换为 OpenCV 格式
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 保存为 PNG（推荐，无损）
        cv2.imwrite(filename, cv_image)
        
        print(f"✅ 图像已保存为 {filename}")
    except Exception as e:
        print(f"❌ 保存失败: {e}")

if __name__ == '__main__':
    main()