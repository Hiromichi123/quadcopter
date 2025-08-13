import torch 
from PIL import Image
import cv2
import yaml
import logging
import os
import sys

# 获取文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 添加cn_clip包的路径（指向源码目录）
if 'build' in current_dir:
    # 在build环境中，指向源码目录
    yolip_root = '/home/jetson/ros2/2025-FCU/src/yolip'
else:
    # 在源码环境中
    yolip_root = os.path.dirname(os.path.dirname(current_dir))
sys.path.append(yolip_root)

import cn_clip.clip as clip
from cn_clip.clip import load_from_name

CHINESE_TO_ENGLISH = {
    '大象': 'elephant',
    '老虎': 'tiger',
    '狼': 'wolf',
    '猴子': 'monkey',
    '孔雀': 'peacock'
}

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

yaml_path = os.path.join(os.path.dirname(current_dir), "list_cn.yaml")
model_dir = yolip_root

# 获取CLIP模型的标签列表（中文）
with open(yaml_path, 'r', encoding='utf-8') as f:
    prompt_words = yaml.load(f.read(), Loader=yaml.FullLoader)

device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = load_from_name("ViT-L-14", device=device, download_root=model_dir)
model.eval()
logger.info("CLIP模型加载成功。")

class ClipResultObj():
    confidence = None
    name = None

    def __init__(self, confidence=None, name=None):
        if confidence != None:
            self.confidence = float(confidence)
        if name:
            self.name = str(name)

# 推理
def infer(img) -> list[ClipResultObj]:
    img = preprocess(Image.fromarray( cv2.cvtColor(img, cv2.COLOR_BGR2RGB) )).unsqueeze(0).to(device)
    text = clip.tokenize(prompt_words).to(device)

    with torch.no_grad():
        image_features = model.encode_image(img)
        text_features = model.encode_text(text)
        # 对特征进行归一化
        image_features /= image_features.norm(dim=-1, keepdim=True) 
        text_features /= text_features.norm(dim=-1, keepdim=True)    

        logits_per_image, logits_per_text = model.get_similarity(img, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        # 找到最大 prob
        probs = list(probs[0])
        results = []
        for i in range(len(probs)):
            if probs[i] > 0.6:
                logger.info(f"clip: {prompt_words[i]} {probs[i]}")
            results.append(
                ClipResultObj(name=prompt_words[i], confidence=probs[i])
            )
        return results

# 绘制CLIP识别种类和置信度
def draw_clip_result(img, yolo_result, name, confidence):
    x1, y1, x2, y2 = map(int, yolo_result.box)
    english_name = CHINESE_TO_ENGLISH.get(name, name)
    text = f"{english_name}: {confidence:.2f}"
    cv2.putText(img, text, (x2 + 10, y1 + 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
