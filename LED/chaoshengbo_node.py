import rospy  
from std_msgs.msg import Int32  
import jetson.GPIO as GPIO  
import time  
  
# 设置GPIO模式为board  
GPIO.setmode(GPIO.BOARD)  
  
# 设定Trig和Echo的物理引脚编号  
TRIG_PIN = 13  # 替换为Trig引脚的实际物理编号  
ECHO_PIN = 15  # 替换为Echo引脚的实际物理编号  
  
# 初始化ROS节点和发布者  
def init_ros_node_and_publisher():  
    rospy.init_node('distance_sensor_node', anonymous=True)  
    pub = rospy.Publisher('MACHAO', Int32, queue_size=10)  
    return pub  
  
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
def main():  
    pub = init_ros_node_and_publisher()  # 初始化ROS节点和发布者  
    rate = rospy.Rate(1)  # 设置发布频率为1Hz  
      
    try:  
        while not rospy.is_shutdown():  
            distance = get_distance()  
              
            if distance < 0.1:  
                print("find it")  
                # 发布消息到话题'MACHAO'  
                pub.publish(114514)  
              
            rate.sleep()  # 按照设定的频率休眠，保证稳定的发布频率  
    except rospy.ROSInterruptException:  
        pass  
    finally:  
        GPIO.cleanup()  # 清理GPIO设置  
  
if __name__ == '__main__':  
    main()