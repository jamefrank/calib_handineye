import os
import json
import transforms3d
import numpy as np
import open3d as o3d

T_cam2gripper = np.array(
    [
        [-0.9910904185208098, 0.006794007825452645, 0.1330173814729248, -0.02630493332592121*1000],
        [-0.005001707076004768, -0.9998922146349077, 0.01380369656379392, 0.06774159592806521*1000],
        [0.1330968265683738, 0.01301539742640003, 0.991017575115225, -0.07437587449366402*1000],
        [0.0, 0.0, 0.0, 1.0]       
    ]
)




input_dir = "/home/frank/Downloads/calib(3)/calib"
output_dir = "/home/frank/Pictures/handeye/output_cloud"

for item in os.listdir(input_dir):
    json_path = os.path.join(input_dir, item, "aubo_robot", "tcp_pose.json")
    ply_path = os.path.join(input_dir, item, "mech_eye", "point_cloud.ply")
    with open(json_path, 'r') as f:
        data = json.load(f)
        tcp_pose = data.get('tcp_pose', [])
        R = transforms3d.euler.euler2mat(tcp_pose[-3], tcp_pose[-2], tcp_pose[-1], axes='sxyz')
        RT = transforms3d.affines.compose(
            T=[tcp_pose[0]*1000, tcp_pose[1]*1000, tcp_pose[2]*1000],      # 平移
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
        