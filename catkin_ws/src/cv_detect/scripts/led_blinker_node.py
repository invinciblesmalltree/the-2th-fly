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
            delta_x = int(M['m10'] / M['m00'])-width/2
            delta_y = int(M['m01'] / M['m00'])-height/2
            return delta_x, delta_y

    return None

def blink_led(times):
    # 设置GPIO模式为board
    GPIO.setmode(GPIO.BOARD)  

    # 设置LED输出引脚
    LED_PIN = 12
    GPIO.setup(LED_PIN, GPIO.OUT)

    for _ in range(times):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(1)

# 初始化节点
rospy.init_node('led_node', anonymous=True)

pub = rospy.Publisher('led_msg', LedMsg, queue_size=10)
rate = rospy.Rate(20)

# cv识别程序主体
capture = cv2.VideoCapture(0)

while(1):
    if capture.isOpened():
        open, frame = capture.read()
        cv2.imshow('frame', frame)

        # 获取视频信息
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

        delta_x, delta_y = detect_blue_objects(frame)

        led_msg = LedMsg()
        led_msg.value = False
        led_msg.delta_x = delta_x
        led_msg.delta_y = delta_y

        pub.publish(led_msg)

        # 保存视频帧到本地便于查看
        # out.write(frame)
        # out.release()
        capture.release()

    rospy.spinOnce()
    rate.sleep()

GPIO.cleanup()