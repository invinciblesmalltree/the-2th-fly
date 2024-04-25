#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from PIL import Image
import jetson.gpio as GPIO
import time

def decode_barcode(image):
    barcodes = decode(image)

    for barcode in barcodes:
        barcode_type = barcode.type
        barcode_data = barcode.data.decode('utf-8')
        print(f"发现条形码：类型：{barcode_type}，数据：{barcode_data}")

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