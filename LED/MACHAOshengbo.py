import jetson.GPIO as GPIO  
import time  
  
# 设置GPIO模式为board  
GPIO.setmode(GPIO.BOARD)  
  
# 设定Trig和Echo的物理引脚编号  
TRIG_PIN = X  # 替换为Trig引脚的实际物理编号  
ECHO_PIN = Y  # 替换为Echo引脚的实际物理编号  
  
# 设置Trig和Echo引脚为输出和输入模式  
GPIO.setup(TRIG_PIN, GPIO.OUT)  
GPIO.setup(ECHO_PIN, GPIO.IN)  
  
def get_distance():  
    # 发送10us的脉冲到Trig引脚  
    GPIO.output(TRIG_PIN, GPIO.HIGH)  
    time.sleep(0.00001)  
    GPIO.output(TRIG_PIN, GPIO.LOW)  
      
    start_time = time.time()  
      
    # 等待Echo引脚变为高电平  
    while GPIO.input(ECHO_PIN) == 0:  
        pass  
      
    # 等待Echo引脚变为低电平  
    while GPIO.input(ECHO_PIN) == 1:  
        pass  
      
    end_time = time.time()  
      
    # 计算时间差  
    pulse_duration = end_time - start_time  
      
    # 计算距离（假设声速为340m/s）  
    distance = (pulse_duration * 34000) / 2  
    return distance  
  
# 主循环  
while True:  
    distance = get_distance()  
      
    if distance < 0.5:  
        print("find it")  
      
    time.sleep(1)  # 每秒检测一次  
  
# 清理GPIO设置  
GPIO.cleanup()