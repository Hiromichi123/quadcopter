from ultralytics import YOLO
from .utils import cut_square
import cv2
import os

# 获取模型文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(os.path.dirname(current_dir), "yolo_cut.pt")
model_cut = YOLO(model_path, verbose=False) 

class YoloResultObj():
    confidence = None
    id = None
    name = None
    square_cut_img = None
    box = None

    def __init__(self, confidence=None, id=None, name=None, square_cut_img=None, box=None):
        if confidence != None:
            self.confidence = float(confidence)
        if id != None:
            self.id = int(id)
        if name:
            self.name = str(name)
        self.square_cut_img = square_cut_img
        self.box = box
    
    # 获取矩形框的中心点
    def get_center_point(self):
        x1, y1, x2, y2 = map(int, self.box[:4])
        center_x = int((x1 + x2)/2)
        center_y = int((y1 + y2)/2)
        return (center_x, center_y)


def infer_cut(img, confidence_threshold: float = 0.5) -> list[YoloResultObj]:
    results = model_cut(img, verbose=False)
    result_list = []

    # 获取图像尺寸和中心区域边界
    img_height, img_width = img.shape[:2]
    center_size = img_width // 3  # 三分之一宽度
    
    # 中心区域边界
    left_bound = (img_width - center_size) // 2
    right_bound = left_bound + center_size
    top_bound = (img_height - center_size) // 2
    bottom_bound = top_bound + center_size

    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return result_list
    
    for i in range(len(boxes)):
        box = boxes.xyxy[i].cpu().numpy().tolist()
        confidence = boxes.conf[i].item()
        cls_id = int(boxes.cls[i].item())
        name = model_cut.names[cls_id]

        # 过滤掉置信度低于阈值的框
        if confidence >= confidence_threshold:
            # 计算检测框中心点
            x1, y1, x2, y2 = box
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            # 判断中心点是否在边界内
            if (left_bound <= center_x <= right_bound and 
                top_bound <= center_y <= bottom_bound):
                r = YoloResultObj(
                    confidence=confidence,
                    id=cls_id,
                    name=name,
                    square_cut_img=cut_square(img, box),
                    box=box,
                )
                result_list.append(r)
    return result_list

# 绘制最中心yolo矩形框+置信度
def draw_yolo_box(img, yolo_result: YoloResultObj, color=(0, 0, 255), thickness=2):
    x1, y1, x2, y2 = map(int, yolo_result.box)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    confidence = yolo_result.confidence
    label = f"{confidence:.2f}"
    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

# 绘制所有yolo矩形框+置信度
def draw_all_yolo_boxes(img, yolo_results: list[YoloResultObj], color=(0, 0, 255), thickness=2):
    if not yolo_results:
        return
    for i, yolo_result in enumerate(yolo_results):
        x1, y1, x2, y2 = map(int, yolo_result.box)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
        confidence = yolo_result.confidence
        name = yolo_result.name if hasattr(yolo_result, 'name') else f"obj_{i}"
        label = f"{name}: {confidence:.2f}"
        cv2.putText(img, label, (x1, y1 - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), thickness)

# 手动测试
if __name__ == "__main__":
    img = cv2.imread("test.jpg", flags=1)
    if img is None:
        raise FileNotFoundError("找不到图片")

    results = infer_cut(img)
    for result in results:
        print(result.id, result.name, result.confidence, type(result.square_cut_img))

        x1, y1, x2, y2 = map(int, result.box)
        label = f"{result.name} {result.confidence:.2f}"

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)

    cv2.imshow("YOLOv8 Detection", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
