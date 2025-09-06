import os
import json
import transforms3d
import numpy as np
import open3d as o3d

T_cam2gripper = np.array(
    [
        [-0.9919855653026715, 0.03136448851834635, 0.1223965158455169, -0.02606969836328629],
        [-0.02911690590336935, -0.9993737169668689, 0.02010919258545293, 0.0654983517604585],
        [0.1229505755242882, 0.01638422093988664, 0.9922775384349078, -0.09256224788067262],
        [0.0, 0.0, 0.0, 1.0]       
    ]
)

T_cam2gripper = np.array(
    [
        [-0.9894488619137813, 0.03148876015898994, 0.1414192619170833, -0.0358034623308598],
        [-0.02745840085128495, -0.9991617606025593, 0.03036136314930907, 0.05900954865508048],
        [0.142256760402496, 0.02615786943242275, 0.9894840978947277, -0.09235931782140221],
        [0.0, 0.0, 0.0, 1.0]       
    ]
)

T_cam2gripper = np.array(
    [
        [-0.9917965571527292, 0.03907904840540369, 0.1217062742660436, -0.03316440907770515],
        [-0.03926239823747045, -0.9992285364347659, 0.0008922214287404833, 0.08215064240860753],
        [0.1216472494741891, -0.003893578066989504, 0.9925657594059959, -0.06417382202021796],
        [0.0, 0.0, 0.0, 1.0]       
    ]
)

T_cam2gripper = np.array(
    [
        [-0.9905106804002215, 0.02499323401114336, 0.1351441092565807, -0.03132403324940991],
        [-0.02422938080360802, -0.999679814459566, 0.007294221545941876, 0.07387686803047194],
        [0.1352831442529488, 0.003950546259918741, 0.9907991037870816, -0.07503147026237017],
        [0.0, 0.0, 0.0, 1.0]       
    ]
)

T_cam2gripper = np.array(
    [
        [-0.9968098649506979, 0.001249466554240737, 0.07980308246115184, -0.007674674969152272],
        [-0.01301196412820597, -0.9890441534794633, -0.1470454054284537, 0.09944200126285128],
        [0.07874504382180408, -0.1476147055730623, 0.9859052270741244, -0.0766219150645322],
        [0.0, 0.0, 0.0, 1.0]       
    ]
)

input_dir = "/home/frank/Downloads/calib(2)/calib"
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
        