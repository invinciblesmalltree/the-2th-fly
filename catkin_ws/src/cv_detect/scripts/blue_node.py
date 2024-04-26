#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
import Jetson.GPIO as GPIO
import time
from cv_detect.msg import LedMsg

def detect_blue_objects(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义蓝色的HSV范围
    lower_blue = np.array([100, 43, 46])
    upper_blue = np.array([124, 255, 255])

    # 提取蓝色区域
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)

        if area > 1000:
            # 找到图形轮廓中心坐标
            M = cv2.moments(contour)
            delta_x = int(M['m10'] / M['m00']-width/2)
            delta_y = -int(M['m01'] / M['m00']-height/2)
            return delta_x, delta_y

    return None

def normal_blink(times):
    # 设置GPIO模式为board
    GPIO.setmode(GPIO.BOARD)  

    # 设置LED输出引脚
    LED_PIN = 11
    GPIO.setup(LED_PIN, GPIO.OUT)

    for _ in range(times):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(0.5)

# 初始化节点
rospy.init_node('blue_node', anonymous=True)

pub = rospy.Publisher('led_msg', LedMsg, queue_size=10)
rate = rospy.Rate(20)

# cv识别程序主体
capture = cv2.VideoCapture('/dev/ground')

while(1):
    if capture.isOpened():
        open, frame = capture.read()
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

        # 获取视频信息
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        led_msg = LedMsg()

        delta = detect_blue_objects(frame)
        if delta is None:
            led_msg.value = False
        else:
            led_msg.value = True
            led_msg.delta_x, led_msg.delta_y= delta

        pub.publish(led_msg)

    rate.sleep()

capture.release()
GPIO.cleanup()