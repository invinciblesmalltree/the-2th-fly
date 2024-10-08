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

def detect_green(image, width):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义绿色的HSV范围
    lower_green = np.array([35, 43, 46])
    upper_green = np.array([77, 255, 255])

    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # 设置红色的低值和高值阈值
    # lower_red1 = np.array([0, 120, 50])
    # upper_red1 = np.array([10, 255, 255])
    # lower_red2 = np.array([160, 120, 50])
    # upper_red2 = np.array([179, 255, 255])

    # # 创建红色区域的掩模
    # mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    # mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    # mask = cv2.bitwise_or(mask1, mask2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        # 取最大的轮廓
        max_contour = max(contours, key=cv2.contourArea)
    
        # 计算轮廓的中心点
        moments = cv2.moments(max_contour)
        if moments['m00'] != 0:  # 防止除以0
            x = int(moments['m10'] / moments['m00'])
            y = int(moments['m01'] / moments['m00'])
            center = (x, y)
            return int(x - width / 2)
    return 2147483647

def decode_barcode(image):
        barcodes = decode(image)
        if len(barcodes) == 0:
            return None
        barcode_data = barcodes[0].data.decode('utf-8')
        return int(barcode_data)


def perprocess(image):
    # 灰度图处理
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 二值化处理
    _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    cv2.imshow('thresh', thresh)
    # 提取轮廓
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if contours is not None:
        for contour in contours:
            # 面积
            area = cv2.contourArea(contour)
            # 边界矩形
            x, y, w, h = cv2.boundingRect(contour)
            if area > 1000 :
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
width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
led_open = False # led连闪开关
pole_detected = False
last_request = rospy.Time.now()
n = -1

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
        bar_msg.delta_x = detect_green(frame, width)
        if not pole_detected:
            bar_msg.n = -1
            if abs(bar_msg.delta_x) < 50:
                pole_detected = True

        if pole_detected and not led_open:
            ret = decode_barcode(frame)
            if ret is None:
                bar_msg.n = -1
            else:
                if ret < 1 or ret > 9:
                    rate.sleep()
                    continue
                led_open = True
                bar_msg.n= ret
                n = ret
                rospy.loginfo('Barcode: %s', ret)

        if led_open:
            bar_msg.n = n

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