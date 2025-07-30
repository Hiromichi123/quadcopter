from ultralytics import YOLO
from .utils import logger, cut_square
import cv2

model_cut = YOLO("yolo_cut.pt") 
logger.info("Yolo_cut模型加载成功。")

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


def infer_cut(img) -> list[YoloResultObj]:
    results = model_cut(img)
    result_list = []

    boxes = results[0].boxes
    for i in range(len(boxes)):
        box = boxes.xyxy[i].cpu().numpy().tolist()
        confidence = boxes.conf[i].item()
        cls_id = int(boxes.cls[i].item())
        name = model_cut.names[cls_id]

        r = YoloResultObj(
            confidence=confidence,
            id=cls_id,
            name=name,
            square_cut_img=cut_square(img, box),
            box=box,
        )
        result_list.append(r)
    
    return result_list

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
