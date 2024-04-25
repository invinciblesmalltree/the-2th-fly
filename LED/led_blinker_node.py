import rospy  
from your_package.msg import BlinkMsg  # 替换your_package为你的包名  
import jetson.gpio as GPIO  
import time  
  
# 初始化ROS节点  
rospy.init_node('led_blinker_node', anonymous=True)  
  
# 创建一个订阅器，订阅led_blink_topic主题，消息类型为BlinkMsg，回调函数为blink_led_callback  
rospy.Subscriber("led_blink_topic", BlinkMsg, blink_led_callback)  
  
# 设置GPIO模式为board  
GPIO.setmode(GPIO.BOARD)  
  
# 设置LED灯连接的物理引脚编号  
LED_PIN = 12  # 替换为LED灯实际连接的物理引脚编号  
  
# 设置LED引脚为输出模式  
GPIO.setup(LED_PIN, GPIO.OUT)  
  
# 回调函数，当接收到消息时调用  
def blink_led_callback(msg):  
    # 使用接收到的num_blinks值来驱动LED闪烁  
    blink_led(msg.num_blinks, 0.5)  
  
# LED闪烁函数  
def blink_led(times, interval):  
    for _ in range(times):  
        # 打开LED灯  
        GPIO.output(LED_PIN, GPIO.HIGH)  
        time.sleep(interval)  
          
        # 关闭LED灯  
        GPIO.output(LED_PIN, GPIO.LOW)  
        time.sleep(interval)  
  
# 保持节点运行，以便能够接收消息  
rospy.spin()  
  
# 在ROS节点停止时清理GPIO设置  
GPIO.cleanup()