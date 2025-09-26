import os
import json
import transforms3d
import numpy as np
import open3d as o3d


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

for item in os.listdir(input_dir):
    json_path = os.path.join(input_dir, item, "aubo_robot", "robot_status.json")
    ply_path = os.path.join(input_dir, item, "mech_eye", "point_cloud.ply")
    with open(json_path, 'r') as f:
        data = json.load(f)
        robot_status = data.get('robot_status', {})
        tcp_pose = robot_status.get('tcp_pose', [])
        R = transforms3d.euler.euler2mat(tcp_pose[-3], tcp_pose[-2], tcp_pose[-1], axes='sxyz')
        RT = transforms3d.affines.compose(
            # T=[tcp_pose[0]*1000, tcp_pose[1]*1000, tcp_pose[2]*1000],      # 平移
            T=[tcp_pose[0], tcp_pose[1], tcp_pose[2]],      # 平移
            R=R,              # 旋转矩阵
            Z=[1.0, 1.0, 1.0] # 缩放（单位缩放）
        )
        RT_final = np.dot(RT, T_cam2gripper)
        print(item)
        print(RT_final)
        
        #
        output_ply_path = os.path.join(output_dir, f"{item}.ply")
        # output_ori_ply_path = os.path.join(output_dir, f"{item}_ori.ply")
        pcd = o3d.io.read_point_cloud(ply_path)
        pcd_transformed = o3d.geometry.PointCloud(pcd)
        pcd_transformed.transform(RT_final)
        original_center = pcd.get_center()
        print(f"变换前中心点: {original_center}")
        transformed_center = pcd_transformed.get_center()
        print(f"变换后中心点: {transformed_center}")
        o3d.io.write_point_cloud(output_ply_path, pcd_transformed)
        # o3d.io.write_point_cloud(output_ori_ply_path, pcd)
        