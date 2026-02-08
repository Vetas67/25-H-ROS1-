import cv2
import numpy as np
import onnxruntime as rt

CONFIDENCE = 0.5

CLASSES = {
    0: 'elsphant',
    1: 'monkey',
    2: 'peacock',
    3: 'tiger',
    4: 'wolf',
}


def box_iou(box1, box2, eps=1e-7):
    (a1, a2), (b1, b2) = box1.unsqueeze(1).chunk(2, 2), box2.unsqueeze(0).chunk(2, 2)
    inter = (np.minimum(a2, b2) - np.maximum(a1, b1)).clip(0).prod(2)
    return inter / ((a2 - a1).prod(2) + (b2 - b1).prod(2) - inter + eps)


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    shape = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)

    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scaleFill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, ratio, (dw, dh)


def onnx_inf(onnxModulePath, data):
    """运行ONNX模型推理"""
    # 创建推理会话
    sess = rt.InferenceSession(onnxModulePath)
    input_name = sess.get_inputs()[0].name
    output_name = sess.get_outputs()[0].name
    
    # 确保数据是正确的形状
    if data.size != 1228800:  # 3*640*640=1228800
        # 尝试调整大小
        data = data.reshape(-1)  # 展平为一维数组
        # 截取或填充到所需长度
        if data.size > 1228800:
            data = data[:1228800]  # 截断
        else:
            # 填充（用0填充）
            data = np.pad(data, (0, 1228800 - data.size), 'constant')
    
    # 重塑为模型期望的形状
    input_data = data.reshape(1, 3, 640, 640).astype(np.float32)
    
    # 运行推理
    pred_onnx = sess.run([output_name], {input_name: input_data})
    
    return pred_onnx


def xywh2xyxy(x):
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2
    y[..., 1] = x[..., 1] - x[..., 3] / 2
    y[..., 2] = x[..., 0] + x[..., 2] / 2
    y[..., 3] = x[..., 1] + x[..., 3] / 2
    return y


def nms_boxes(boxes, scores):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= 0.45)[0]

        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def non_max_suppression(
        prediction,
        conf_thres=0.25,
        iou_thres=0.45,
        classes=None,
        agnostic=False,
        multi_label=False,
        labels=(),
        max_det=300,
        nm=0,
):
    assert 0 <= conf_thres <= 1, f'Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0'
    assert 0 <= iou_thres <= 1, f'Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0'
    if isinstance(prediction, (list, tuple)):
        prediction = prediction[0]

    bs = prediction.shape[0]
    nc = prediction.shape[2] - nm - 5
    xc = prediction[..., 4] > conf_thres

    max_wh = 7680
    max_nms = 30000
    redundant = True
    multi_label &= nc > 1
    merge = False

    mi = 5 + nc
    output = [np.zeros((0, 6 + nm))] * bs

    for xi, x in enumerate(prediction):
        x = x[xc[xi]]
        if labels and len(labels[xi]):
            lb = labels[xi]
            v = np.zeros((len(lb), nc + nm + 5))
            v[:, :4] = lb[:, 1:5]
            v[:, 4] = 1.0
            v[range(len(lb)), lb[:, 0].astype(int) + 5] = 1.0
            x = np.concatenate((x, v), 0)

        if not x.shape[0]:
            continue

        x[:, 5:] *= x[:, 4:5]

        box = xywh2xyxy(x[:, :4])
        mask = x[:, mi:]

        if multi_label:
            i, j = (x[:, 5:mi] > conf_thres).nonzero()
            x = np.concatenate((box[i], x[i, 5 + j, None], j[:, None].astype(float), mask[i]), 1)
        else:
            conf = np.max(x[:, 5:mi], 1).reshape(box.shape[0], 1)
            j = np.argmax(x[:, 5:mi], 1).reshape(box.shape[0], 1)
            x = np.concatenate((box, conf, j, mask), 1)[conf.reshape(box.shape[0]) > conf_thres]

        if classes is not None:
            x = x[(x[:, 5:6] == np.array(classes)).any(1)]

        n = x.shape[0]
        if not n:
            continue
        index = x[:, 4].argsort()[:max_nms][::-1]
        x = x[index]

        c = x[:, 5:6] * (0 if agnostic else max_wh)
        boxes, scores = x[:, :4] + c, x[:, 4]
        i = nms_boxes(boxes, scores)
        i = i[:max_det]

        if merge and (1 < n < 3E3):
            iou = box_iou(boxes[i], boxes) > iou_thres
            weights = iou * scores[None]
            x[i, :4] = np.multiply(weights, x[:, :4]) / weights.sum(1, keepdims=True)
            if redundant:
                i = i[iou.sum(1) > 1]

        output[xi] = x[i]

    return output


def clip_boxes(boxes, shape):
    boxes[..., [0, 2]] = boxes[..., [0, 2]].clip(0, shape[1])
    boxes[..., [1, 3]] = boxes[..., [1, 3]].clip(0, shape[0])


def scale_boxes(img1_shape, boxes, img0_shape, ratio_pad=None):
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    boxes[..., [0, 2]] -= pad[0]
    boxes[..., [1, 3]] -= pad[1]
    boxes[..., :4] /= gain
    clip_boxes(boxes, img0_shape)
    return boxes


onnxModulePath = "animal_detector.onnx"
imgsz = (640, 640)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    cap = cv2.VideoCapture(1)
    print("failed to open camera 0,change to camera 1")

category_count = []

def detect(detection_num):
    for count in range(1,detection_num):
        ret, frame = cap.read()
        if not ret:
            return
    
        # preprocess
        im = letterbox(frame, imgsz, auto=True)[0]
        im = im.transpose((2, 0, 1))[::-1]
        im = np.ascontiguousarray(im)
        im = im.astype(np.float32)
        im /= 255
        if len(im.shape) == 3:
            im = im[None]
    
        # inference
        pred = onnx_inf(onnxModulePath, im)
    
        # NMS
        conf_thres = 0.25
        iou_thres = 0.45
        max_det = 1000
        classes = None
        agnostic_nms = False
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        
    
        # Process predictions
        for i, det in enumerate(pred):
            if len(det):
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], frame.shape).round()
                for *xyxy, conf, cls in det:
                    x1, y1, x2, y2 = map(int, xyxy)
                    cls = int(cls)
                    prob = float(conf)
                    if prob >= CONFIDENCE:
                        # 更新计数数组数据
                        attend_flag = 1           
                        for it_count in range(len(category_count)):
                            item = category_count[it_count]
                            if((item['category'] == CLASSES[cls]) and ((item['PX'] - (x1+x2)/2)*(item['PX'] - (x1+x2)/2) + (item['PY'] - (y1+y2)/2)*(item['PY'] - (y1+y2)/2) < 50000)):
                                category_count[it_count]['count']+=1
                                category_count[it_count]['PX'] = (category_count[it_count]['PX'] + (x1+x2)/2) / category_count[it_count]['count']
                                category_count[it_count]['PY'] = (category_count[it_count]['PY'] + (y1+y2)/2) / category_count[it_count]['count']
                                attend_flag = 0
                                break
                                
                        if(attend_flag == 1):
                            category_count.append({
                                "category":CLASSES[cls],
                                "PX":(x1+x2)/2,
                                "PY":(y1+y2)/2,
                                "count":1
                            })
        
        cv2.imshow("YOLO",frame)
                            
while(True):
    category_count.clear()
    detect(8)
    for res in category_count:
        if(res['count'] >= 1):
            print(f"{res['category']} {int(res['PX'])} {int(res['PY'])}")
    
cap.release()
cv2.destroyAllWindows()