import numpy as np

def edit_map(input_file, output_file):
    with open(input_file, 'rb') as f:
        # 读取 PGM 头部
        header = f.readline() # P5
        comment = b""
        line = f.readline()
        while line.startswith(b'#'):
            comment += line
            line = f.readline()
        
        # 获取分辨率
        width, height = map(int, line.split())
        max_val = int(f.readline())
        
        # 读取像素数据
        data = np.fromfile(f, dtype=np.uint8).reshape((height, width))

    # 假设分辨率是 0.05m/pixel (Nav2 默认)
    # 5.0m 对应 100 像素
    res = 0.05
    
    # 【定义禁区坐标 - 单位：米】
    # Y 轴区间 (根据你的代码 1.9m ~ 2.4m)
    y_min, y_max = 1.9, 2.4
    # X 轴区间 (左右两块，中间留出约 1m 的通道)
    # 左块: 0 到 2.0m; 右块: 3.0m 到 5.0m
    zones = [
        {'x': (0.0, 2.0), 'y': (y_min, y_max)},
        {'x': (3.0, 5.0), 'y': (y_min, y_max)}
    ]

    # 将米转换为像素索引 (注意：PGM 坐标原点通常在左上角)
    # 如果你的原点在左下角，需要进行 height - y 的转换
    for zone in zones:
        x_start = int(zone['x'][0] / res)
        x_end = int(zone['x'][1] / res)
        # 注意：Nav2 地图 Y 轴通常是反的，这里我们涂抹对应高度区间
        # 实际操作中建议在 RViz 查看坐标后微调
        y_start = height - int(zone['y'][1] / res)
        y_end = height - int(zone['y'][0] / res)
        
        # 将该区域填充为 0 (纯黑色，代表障碍物)
        data[y_start:y_end, x_start:x_end] = 0

    # 保存新地图
    with open(output_file, 'wb') as f:
        f.write(b'P5\n')
        f.write(comment)
        f.write(f'{width} {height}\n{max_val}\n'.encode())
        data.tofile(f)
    print(f"成功！已生成黑化禁区地图: {output_file}")

if __name__ == "__main__":
    # 请确保文件名正确
    edit_map('ai_map.pgm', 'ai_map_fixed.pgm')