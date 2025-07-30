import torch 
from PIL import Image
import cv2
from .utils import logger, get_clip_list

import cn_clip.clip as clip
from cn_clip.clip import load_from_name

prompt_words = get_clip_list()

device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = load_from_name("ViT-L-14", device=device, download_root='./')
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
            if probs[i] > 0.2:
                logger.info(f"clip: {prompt_words[i]} {probs[i]}")
            results.append(
                ClipResultObj(name=prompt_words[i], confidence=probs[i])
            )
        return results
