import cv2
import numpy as np
from pyzbar.pyzbar import decode
from PIL import Image

def detect_blue_objects(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 蓝色的HSV
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])

    # 蓝色掩模
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 除噪
    mask = cv2.dilate(mask, (3, 3), iterations=2)
    mask = cv2.erode(mask, (3,3), iterations=2)

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)

        if area > 1000:
            # 找到图形轮廓中心坐标
            M = cv2.moments(contour)
            center_x = int(M['m10'] / M['m00'])
            center_y = int(M['m01'] / M['m00'])
            center = (center_x, center_y)
            return center

    return None

def decode_barcode(image):
    barcodes = decode(image)

    for barcode in barcodes:
        barcode_type = barcode.type
        barcode_data = barcode.data.decode('utf-8')
        print(f"发现条形码：类型：{barcode_type}，数据：{barcode_data}")



# cv识别程序主体
capture = cv2.VideoCapture(0)
# 设置摄像头分辨率为720p/1080p
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# 检查是否正确打开
if capture.isOpened():
    open, frame = capture.read()

    # 获取视频信息
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS)
    out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

    ret, frame = capture.read()
    frame=ROI
    if case == 1: # 调用检测函数
        object_center = detect_blue_objects(frame)
        if object_center is not None:
            print(str(object_center[0]) + str(object_center[1]))  # 打印蓝色物体中心
    elif case == 2: # 调用扫码函数
        decode_barcode(frame)


    # 保存视频帧到本地便于查看
    cv2.putText(frame, "FPS:" + str(fps), (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
    out.write(frame)
    out.release()
    capture.release()