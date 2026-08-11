import cv2
import numpy as np

# ================= 配置区 =================
# 1. YAML 里的原始参数
RESOLUTION = 0.05
ORIGIN_X = -10.0
ORIGIN_Y = -10.0
INPUT_MAP = 'ai_map2.pgm'
OUTPUT_MAP = 'ai_map_fixed.pgm'

# 2. 偏移微调（如果你发现黑块偏左了，就增加 X_NUDGE；偏下就增加 Y_NUDGE）
# 请根据你在 RViz 里看到的偏差大小，以“米”为单位调整
X_NUDGE = -0.8  # 左右偏移
Y_NUDGE = 0.0  # 上下偏移

# 3. 禁区尺寸定义（米）
# [左侧矩形 x1, y1, x2, y2], [右侧矩形 x1, y1, x2, y2]
# 这里的坐标是在场地中的位置，我们会加上偏移量
ZONES = [
    [0.0, 1.9, 2.0, 2.4], # 左侧
    [3.0, 1.9, 5.0, 2.4]  # 右侧
]
# ==========================================

def world_to_pixel(x, y, h):
    # 应用偏移量并转换
    u = int((x + X_NUDGE - ORIGIN_X) / RESOLUTION)
    v = int((y + Y_NUDGE - ORIGIN_Y) / RESOLUTION)
    return u, h - 1 - v

def process():
    img = cv2.imread(INPUT_MAP, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("错误：找不到 ai_map.pgm")
        return
    
    h, w = img.shape
    
    # 涂黑操作
    for z in ZONES:
        u1, v1 = world_to_pixel(z[0], z[1], h)
        u2, v2 = world_to_pixel(z[2], z[3], h)
        # 画黑色实心矩形
        cv2.rectangle(img, (u1, v1), (u2, v2), 0, -1)
        
    cv2.imwrite(OUTPUT_MAP, img)
    print("成功生成 ai_map_fixed.pgm")
    print(f"当前偏移量：X={X_NUDGE}, Y={Y_NUDGE}")

if __name__ == "__main__":
    process()