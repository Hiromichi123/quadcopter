import torch
from PIL import Image
import cv2
import cn_clip.clip as clip
from cn_clip.clip import load_from_name

prompt_words = ["大象", "老虎", "狼", "猴子", "孔雀"]
device = "cuda" if torch.cuda.is_available() else "cpu"
text = clip.tokenize(prompt_words).to(device) # 文本编码

# 从当前目录加载 clip_cn_vit-l-14.pt
model, preprocess = load_from_name("ViT-L-14", device=device, download_root='./')
model.eval()

# 测试图像
img = cv2.imread("test.jpg")
img = preprocess(Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))).unsqueeze(0).to(device)

# 推理
with torch.no_grad():
    image_features = model.encode_image(img)
    text_features = model.encode_text(text)
    image_features /= image_features.norm(dim=-1, keepdim=True)
    text_features /= text_features.norm(dim=-1, keepdim=True)
    logits_per_image, _ = model.get_similarity(img, text)
    probs = logits_per_image.softmax(dim=-1).cpu().numpy()

# 输出结果
probs = list(probs[0])
for i in range(len(probs)):
    if probs[i] > 0.2:
        print(f"[CLIP] {prompt_words[i]}: {probs[i]:.4f}")
