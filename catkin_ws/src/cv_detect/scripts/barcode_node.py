#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from PIL import Image
import Jetson.GPIO as GPIO
import time
from cv_detect.msg import BarMsg

def decode_barcode(image):
    v1=cv2.Canny(image,120,250) # TODO:阈值待调

    if perprocess(image) is not None:
        x, y, w, h = perprocess(image)
        pst1 = np.float32([[x, y], [x+w, y], [x, y+h], [x+w, y+h]]) #顺序左上、右上、左下、右下
        pst2 = np.float32([[0, 0], [150, 0], [0, 300], [150, 300]])
        M = cv2.getPerspectiveTransform(pst1, pst2) # 变换矩阵
        image = cv2.warpPerspective(frame, M, (frame.shape[1], frame.shape[0]))

        barcodes = decode(image)
        if len(barcodes) == 0:
            return None
        barcode_type = barcodes[0].type
        barcode_data = barcodes[0].data.decode('utf-8')
        return int(barcode_data)


def perprocess(image):
    # 灰度图处理
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 二值化处理
    _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    # 提取轮廓
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if contours is not None:
        for contour in contours:
            # 面积
            area = cv2.contourArea(contour)
            # 边界矩形
            x, y, w, h = cv2.boundingRect(contour)
            if area > 1000 and h / w > 1 :
                return x, y, w, h
            
    return None

def blink_led(times):
    # 设置GPIO模式为board
    GPIO.setmode(GPIO.BOARD)  

    # 设置LED输出引脚
    LED_PIN = 11
    GPIO.setup(LED_PIN, GPIO.OUT)

    for _ in range(times):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(0.25)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(0.25)

# 初始化节点
rospy.init_node('barcode_node', anonymous=True)

pub = rospy.Publisher('barcode_msg', BarMsg, queue_size=10)
rate = rospy.Rate(20)

# cv识别程序主体
capture = cv2.VideoCapture('/dev/ahead')
led_open = False # led连闪开关
last_request = rospy.Time.now()

while(1):
    if capture.isOpened():
        open, frame = capture.read()
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

        # 获取视频信息
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

        bar_msg = BarMsg()

        if not led_open:
            ret = decode_barcode(frame)
            if ret is None:
                bar_msg.n = -1
            else:
                if ret < 1 or ret > 9:
                    rate.sleep()
                    continue
                led_open = True
                bar_msg.n= ret
                rospy.loginfo('Barcode: %s', ret)
                cv2.destroyAllWindows()

            pub.publish(bar_msg)

        # 每10秒闪烁1轮
        if led_open and rospy.Time.now()-last_request > rospy.Duration(5):
            blink_led(ret)
            last_request = rospy.Time.now()

        # 保存视频帧到本地便于查看
        # out.write(frame)
        # out.release()

    rate.sleep()

capture.release()
GPIO.cleanup()