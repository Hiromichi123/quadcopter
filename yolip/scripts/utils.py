import yaml
from copy import copy
import logging
import math

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 获取CLIP模型的标签列表（中文）
def get_clip_list():
    with open('list_en.yaml', 'r', encoding='utf-8') as f:
        list_en = yaml.load(f.read(), Loader=yaml.FullLoader)
    with open('list_cn.yaml', 'r', encoding='utf-8') as f:
        list_cn = yaml.load(f.read(), Loader=yaml.FullLoader)
    return list_cn

# 将yolo框转为正方形图像
def cut_square(img, box):
    x1, y1, x2, y2 = map(int, box[:4])
    img_h, img_w = img.shape[:2]
    side = max([x2-x1, y2-y1]) + 10
    if side >= img_h:
        side = img_h

    center_x = int((x1 + x2)/2)
    center_y = int((y1 + y2)/2) 
    sx1 = int(center_x - side/2)
    sy1 = int(center_y - side/2)
    sx2 = int(center_x + side/2)
    sy2= int(center_y + side/2)
    # 左边界
    shift_x, shift_y = 0, 0
    if sx1 < 0:
        shift_x = copy(-sx1)
    # 右边界
    if sx2 > img_w:
        shift_x = copy(img_w-sx2)
    # 上边界
    if sy1 < 0:
        shift_y = copy(-sy1)
    if sy2 > img_h:
        shift_y = copy(img_h-sy2)

    sx1 += shift_x
    sx2 += shift_x
    sy1 += shift_y
    sy2 += shift_y  

    square_cut_img = img[sy1:sy2, sx1:sx2]
    return square_cut_img

# 找到距离相机中心最近的矩形框
def find_nearest(results, image_shape):
    result = None
    min_distance = float('inf')
    h, w = image_shape[:2]
    center_x, center_y = w / 2, h / 2
    for r in results:
        cx, cy = r.get_center_point()
        distance = math.hypot(cx - center_x, cy - center_y)
        if distance < min_distance:
            min_distance = distance
            result = r

    return result
