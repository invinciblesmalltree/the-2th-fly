import jetson.gpio as GPIO  
import time  
  
# 设置GPIO模式为board  
GPIO.setmode(GPIO.BOARD)  
  
# 设置LED灯连接的物理引脚编号  
LED_PIN = 12  # 替换为LED灯实际连接的物理引脚编号  
  
# 设置LED引脚为输出模式  
GPIO.setup(LED_PIN, GPIO.OUT)  
# times：LED灯需要闪烁的次数，默认值为5次。
# interval：每次LED灯打开和关闭之间的时间间隔，以秒为单位，默认值为1秒。 
def blink_led(times, interval):  
    for _ in range(times):  
        # 打开LED灯  
        GPIO.output(LED_PIN, GPIO.HIGH)  
        time.sleep(interval)  
          
        # 关闭LED灯  
        GPIO.output(LED_PIN, GPIO.LOW)  
        time.sleep(interval)  
  
# 调用函数使LED灯闪烁,闪烁n次，间隔1秒  
blink_led(n,1)  
  
# 清理GPIO设置  
GPIO.cleanup()


# import jetson.gpio as GPIO  
# import time  
  
# def blink_led(gpio_pin, blink_times=5, blink_interval=1):  
#     # 设置GPIO模式为BCM  
#     GPIO.setmode(GPIO.BCM)  
      
#     # 设置GPIO引脚为输出模式  
#     GPIO.setup(gpio_pin, GPIO.OUT)  
      
#     for _ in range(blink_times):  
#         # 打开LED灯  
#         GPIO.output(gpio_pin, GPIO.HIGH)  
#         time.sleep(blink_interval)  
          
#         # 关闭LED灯  
#         GPIO.output(gpio_pin, GPIO.LOW)  
#         time.sleep(blink_interval)  
      
#     # 清理GPIO设置  
#     GPIO.cleanup()  
  
# # 使用示例：假设你的LED灯连接到了BCM模式下的GPIO 18  
# blink_led(18)