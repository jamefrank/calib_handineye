import math

# 原始数据：每行前3个是毫米，后3个是度
data = [
    [177.5, -29.6, 357.7, -118.56, -0.33, -77.89],
    [152.7, -95.8, 357.3, -136.24, -48.26, -62.71],
    [133.1, -64.3, 383.3, -116.71, 56.6, -112.42],
    [9.1, -65.3, 325.0, 129.84, -54.27, 55.7],
    [75.8, -64.2, 387.0, 130.79, 54.76, 125.0],
    [101.6, -63.8, 254.2, 102.25, -10.58, 87.51]
]

# 输出文件名
filename = "arm_pose.txt"

# 写入文件
with open(filename, 'w') as f:
    for row in data:
        # 转换：前3个 → 米（/1000），后3个 → 弧度（deg to rad）
        converted_row = [
            val / 1000.0 if i < 3 else math.radians(val)  # 后三个转弧度
            for i, val in enumerate(row)
        ]
        # 格式化输出：每个数保留6位小数，空格分隔
        line = ' '.join(f"{val:10.6f}" for val in converted_row)
        f.write(line + '\n')

print(f"数据已成功保存到 {filename}")
print("前三个：毫米 → 米；后三个：度 → 弧度")