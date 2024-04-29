// 7/22 17:49
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
//geometry_msgs
#include <geometry_msgs/PoseStamped.h>          //定义了一个带时间戳的位姿消息，通常用于描述一个对象在三维空间中的位置和方向
#include <geometry_msgs/Twist.h>                //定义了线速度和角速度的消息，常用于指定机器人的移动速度
#include <geometry_msgs/TwistStamped.h>         //定义了一个带时间戳的 Twist 消息，用于提供速度信息的同时标明该信息的时间点
//mavros_msgs
#include <mavros_msgs/SetMode.h>                //定义了一个服务，用于设置飞行器的飞行模式
#include <mavros_msgs/State.h>                  //定义了飞行器当前状态的消息，包括模式、是否武装等信息
#include <mavros_msgs/CommandBool.h>            //定义了一个服务，该服务发送布尔值命令
#include <mavros_msgs/CommandTOL.h>             //定义了一个服务，用于发送起飞、降落等命令
#include <mavros_msgs/PositionTarget.h>         //定义了一个消息，用于描述目标位置、速度、加速度等信息，常用于控制飞行器的移动
//std_msgs
#include <std_msgs/ColorRGBA.h>                 //定义了RGBA颜色模型的消息，用于表示颜色信息
#include <std_msgs/Int8.h>                      //定义了一个8位整数的消息
#include <std_msgs/Int16.h>                     //定义了一个16位整数的消息
#include <std_msgs/Float32.h>                   //定义了一个32位浮点数的消息
#include <std_msgs/Float32MultiArray.h>         //定义了一个包含多个32位浮点数的数组消息
#include <std_msgs/String.h>                    //定义了一个字符串消息
//tf2和tf
#include <tf2/LinearMath/Quaternion.h>          //提供四元数的定义和操作，用于描述和处理三维旋转
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>//提供了将 geometry_msgs 中的消息类型转换为 tf2 中的数据类型的功能
#include <tf/transform_datatypes.h>             //tf 是处理机器人坐标变换的库，这个头文件提供了多种数据类型，用于处理三维空间中的变换
//一般库
#include <math.h>
#include <std_srvs/Trigger.h>                   //定义了一个简单的无参数服务，该服务可用于触发某些操作，服务的响应包含成功标志和可选的消息
#include <stdlib.h>                             //提供了内存分配、程序控制、转换等操作的函数
//宏参列表
#define TargeyHeight 1.0
#define TargetYaw 0.2
#define TargetRadius 0.5
#define Limitv_x 0.5
#define Limitv_y 0.5
#define Limitv_z 0.5
#define Limit_yaw 0.5

//订阅器列表
ros::Subscriber state_sub; //状态信息
ros::Subscriber pose_sub;  //位置信息
ros::Subscriber vel_sub;   //速度信息
ros::Subscriber lidar_node_sub;  //雷达节点信息
ros::Subscriber depth_data_sub;  //深度信息
ros::Subscriber vision_node_sub; //视觉节点信息
ros::Subscriber land_node_sub;   //着陆节点信息
//发布器列表
ros::Publisher local_pose_pub;   //设置目标位置（全局坐标）
ros::Publisher vel_pub;          //设置速度（全局坐标）
ros::Publisher state_pub;        //状态信息
ros::Publisher set_raw_pub;      //以飞机自己为坐标系的速度控制
ros::Publisher hit_pub;          //未知话题
ros::Publisher laser_pub;        //激光信息
ros::Publisher state_message_pub;//状态消息
//服务客户端列表
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
//消息列表
geometry_msgs::TwistStamped vel_cmd;      //全局消息，速度设置
geometry_msgs::PoseStamped pose,land_pose;//全局消息，分别用来控制飞机自身的位置与飞机降落add_executable(pole_test src/pole_test.cpp)

//部分全局参数
int land_distance = -1;
double last_error_yaw=0, last_error_dist=0;
double yaw_i = 0 , dist_i = 0;

//两个功能函数声明
double min(double a,double b);//取小函数
double arc_normalize_yaw(double current_yaw);//角度值归一化


bool cur_state; //当前状态
bool state_cb(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){  //用于通过命令启动飞机
    cur_state=!cur_state;
    // 显示请求数据
    ROS_INFO("Publish command [%s]", cur_state==true?"Yes":"No");
	// 设置反馈数据
	res.success = true;
	res.message = "Change drone command state!";
    return true;
}

mavros_msgs::State current_state;  //订阅消息：当前状态
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose; //订阅消息：当前位置（世界坐标系）
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

geometry_msgs::PoseStamped orig;

geometry_msgs::TwistStamped current_vel; //订阅消息：当前速度
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    //ROS_INFO("Current velocity: %.2f m/s", msg->twist.linear.x);
    current_vel = *msg;
}

std_msgs::Float32MultiArray lidar_data; //雷达信息的回调函数
void lidar_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    lidar_data = *msg;
}

std_msgs::ColorRGBA vision_data; //视觉信息回调函数
void vision_cb(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    vision_data = *msg;
}

std_msgs::ColorRGBA error_to_target; //目标误差信息回调函数
void land_node_cb(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    error_to_target = *msg;
}

/*------------------分------割------线------------------*/

//世界坐标系下的飞行控制
class move_map{
private:
    double kpx,kpy,kpz; //控制比例系数
    double kix,kiy,kiz; //控制积分系数
    double kdx,kdy,kdz; //控制微分系数
    double range,base;  //用于变速积分的参数
    double kp; //PID控制器的比例系数
    double ki; //PID控制器的积分系数
    double kd; //PID控制器的微分系数
    double last_x, last_y, last_z;    //上一次的位置误差
    double last_vx, last_vy, last_vz; //上一次的速度误差
    double ix, iy ,iz; //积分项
    double acceleration_limit; //加速度限制

    //变速积分的子函数，主要就是对于积分的判断
    double func(double error,double A,double B){
        double result;
        /*当误差error大于等于B+A时，意味着误差超出了系统可以接受的范围，
          此时将积分结果设为0，可以避免积分项对系统产生过大的影响，防止系统过度调节*/
        if(error>=B+A)
        {
            result = 0;
        }
        /*当误差error在B到B+A之间时，说明误差在一个可接受范围内，但仍需要进行积分调节。
        此时根据(A+B-error)/A的计算方式，可以实现一个根据误差大小线性变化的积分结果，
        使得在误差逐渐减小的过程中，积分项逐渐增加*/
        else if(error<B+A && error>B)
        {
            result = (A+B-error)/A;
        }
        /*当误差error小于等于B时，表示误差已经在可接受范围内或者已经被控制在目标范围内，
        此时将积分结果设为1，表示不需要再进行积分调节*/
        else
        {
            result = 1;
        }
        return result;
    }

public:
    //PID飞行控制函数
    int fly_to_target(geometry_msgs::PoseStamped target_pose,ros::Publisher& local_vel_pub, ros::Rate& rate)
    {
        int judge=0;
        // 初始化PID控制器
        double ex = 0.0, ey = 0.0, ez = 0.0;
        // 计算PID控制器输出
        double dt = rate.expectedCycleTime().toSec();//获取 ROS 时间周期并将其转换为秒数，以便在飞行控制中用于计算时间间隔 dt
        ex = target_pose.pose.position.x - current_pose.pose.position.x;
        ey = target_pose.pose.position.y - current_pose.pose.position.y;
        ez = target_pose.pose.position.z - current_pose.pose.position.z;
        ix += ex * dt * func(ex,range,base);
        iy += ey * dt * func(ey,range,base);
        iz += ez * dt * func(ez,range,base);
        double dx = (ex - last_x) / dt;
        double dy = (ey - last_y) / dt;
        double dz = (ez - last_z) / dt;
        double vx = kp * ex + ki * ix + kd * dx;
        double vy = kp * ey + ki * iy + kd * dy;
        double vz = kp * ez + ki * iz + kd * dz;
        last_x = ex;
        last_y = ey;
        last_z = ez;
        // 更新速度指令
        geometry_msgs::TwistStamped vel;
        vel.header.stamp = ros::Time::now();
        vel.twist.linear.x = min(vx,Limitv_x);
        vel.twist.linear.y = min(vy,Limitv_y);
        vel.twist.linear.z = min(vz,Limitv_z);
        vel.twist.angular.x = 0.0;
        vel.twist.angular.y = 0.0;
        vel.twist.angular.z = 0.0;
        // 发布速度指令
        local_vel_pub.publish(vel);
        // 判断是否到达目标点
        if (fabs(ex) < 0.2 && fabs(ey) < 0.2 )
        {
            judge = 1;
        }
        return judge;
}

    //公有接口，设置 PID 控制器的参数
    void set_pid_all(double p,double i,double d){
        kp = p;
        ki = i;
        kd = d;
    }

    //公有接口，初始化 PID 控制器的相关变量
    void pid_init(){
        last_x = 0.0, last_y = 0.0, last_z = 0.0;
        ix = 0.0, iy = 0.0, iz = 0.0; 
        last_vx = 0.0, last_vy = 0.0, last_vz = 0.0;
    }

    //公有接口，设置速度限制
    void set_acceleration_limit(double a){
        acceleration_limit = a;
    }

    //用于打印当前姿态xyz信息
    void printpose(){
        ROS_INFO("px = %.2lf,py = %.2lf,pz = %.2lf",
        current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
    }
};

//局部坐标系下的飞行控制
class move_local{
public:
    //设置飞机的速度，其中x,y,z都是飞机以自己为坐标系的速度，yaw_rate是偏航角旋转速率
    void set_speed_body(double x, double y, double yaw_rate ,ros::Rate& rate)
    {
        mavros_msgs::PositionTarget raw_target; //用于存储速度和姿态信息
        raw_target.coordinate_frame = 1;
        // raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
        // mavros_msgs::PositionTarget::IGNORE_PY | 
        // mavros_msgs::PositionTarget::IGNORE_PZ | 
        // mavros_msgs::PositionTarget::IGNORE_AFX | 
        // mavros_msgs::PositionTarget::IGNORE_AFY | 
        // mavros_msgs::PositionTarget::IGNORE_AFZ ;
        static double last_error=0;
        static double ei;
        double kp=1,ki=0.05,kd=1;
        double error = TargeyHeight - current_pose.pose.position.z;
        double dt = rate.expectedCycleTime().toSec();
        ei += error*dt;
        ei = min(ei,1);
        double ed = (error-last_error)*dt;
        double z = error*kp + ei*ki + kd*ed;
        if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        raw_target.velocity.x = min(x,Limitv_x);
        raw_target.velocity.y = min(y,Limitv_y);
        raw_target.velocity.z = min(z,Limitv_z);
        raw_target.yaw_rate =min(yaw_rate*0.01745329,Limit_yaw);//yaw_rate * 0.01745329;
        ROS_INFO("x = %.2lf y = %.2lf z = %.2lf yaw = %.2lf",raw_target.velocity.x,raw_target.velocity.y,
        raw_target.velocity.z,raw_target.yaw_rate);
        set_raw_pub.publish(raw_target);
    }

    //降落程序中，pid_cal的序号为0
    void velocity_land(ros::Rate& rate){
        mavros_msgs::PositionTarget raw_target;
        raw_target.coordinate_frame = 1;
        raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
        mavros_msgs::PositionTarget::IGNORE_PY | 
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_YAW;
        raw_target.velocity.x = -current_pose.pose.position.x;
        raw_target.velocity.y = -current_pose.pose.position.y;
        raw_target.velocity.z = -current_pose.pose.position.z - 0.2;
        //ROS_INFO("%.2lf",raw_target.velocity.z);
        set_raw_pub.publish(raw_target);
        }

};

//设置飞机的速度，其中x,y,z都是飞机以自己为坐标系的速度，yaw_rate是偏航角旋转速率
void set_speed_body(double x, double y, double yaw_rate ,ros::Rate& rate){
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = 8; //body坐标系
	// raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
    // mavros_msgs::PositionTarget::IGNORE_PY | 
    // mavros_msgs::PositionTarget::IGNORE_PZ | 
    // mavros_msgs::PositionTarget::IGNORE_AFX | 
    // mavros_msgs::PositionTarget::IGNORE_AFY | 
    // mavros_msgs::PositionTarget::IGNORE_AFZ ;
    static double last_error=0;
    static double ei;
    double kp=1,ki=0.05,kd=1;
    double error = TargeyHeight - current_pose.pose.position.z;
    double dt = rate.expectedCycleTime().toSec();
    ei += error*dt;
    ei = min(ei,1);
    double ed = (error-last_error)*dt;
    double z = error*kp + ei*ki + kd*ed;
	raw_target.velocity.x = min(x,Limitv_x);
	raw_target.velocity.y = min(y,Limitv_y);
    raw_target.velocity.z = min(z,Limitv_z);
    raw_target.yaw_rate = min(yaw_rate*0.01745329,Limit_yaw);
	//raw_target.yaw_rate =min(yaw_rate*0.01745329,Limit_yaw);//yaw_rate * 0.01745329;
    ROS_INFO("x = %.2lf y = %.2lf z = %.2lf yaw = %.2lf",raw_target.velocity.x,raw_target.velocity.y,
    raw_target.velocity.z,raw_target.yaw_rate);
	set_raw_pub.publish(raw_target);
}

/*------------------分------割------线------------------*/

//主程序起点
int main(int argc, char **argv)
{
    int step=1;
    double target_speed=0.5;
    int rotation_direction=0;
    std_msgs::ColorRGBA lidar_data_sender;
    double* pole_data;
    geometry_msgs::PoseStamped land_pose;
    double yaw_angle,start_yaw;
    std_msgs::Int8 vision_cmd_sender;
    std_msgs::Int16 led_data_sender;
    led_data_sender.data = 0;
    vision_cmd_sender.data = 1;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;   
    
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);  //订阅状态
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb); //订阅位置信息.
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped> ("/mavros/local_position/velocity", 10 ,vel_cb);
    depth_data_sub = nh.subscribe<std_msgs::ColorRGBA>("/vision/pole/distance",10,depth_cb);
    vision_node_sub = nh.subscribe<std_msgs::Int8>  //降落时找圆用("/vision/pole/number", 10, number_cb);
    local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);  //设置位置
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);  //设定速度
    set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    laser_pub = nh.advertise<std_msgs::Int8>("/offboard/laser",10);
    state_message_pub = nh.advertise<std_msgs::Int8>("/offboard_node/state",10);
    // 定义服务客户端，用于解锁/上锁无人机和切换到离线控制模式
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");   
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    ros::ServiceServer state_client = nh.advertiseService("/command",state_cb);    

    ros::Rate rate(50.0);  

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    mavros_msgs::CommandTOL land_cmd;
    std_msgs::Int8 offboard_state;
    geometry_msgs::PoseStamped target_position;
    geometry_msgs::PoseStamped landing_position;
    double *target_position_ptr;
    land_cmd.request.altitude = 0.0;
    land_cmd.request.yaw = 0.0;
    int flag = 0;
    double radius = 1.0;
    double speed = 0.5;
    double angular_speed = speed / radius;
    double angle = 0.0;
    
    move_map move1;
    move_local move2;

    while(ros::ok())
    {
        ros::spinOnce();

        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_WARN("Offboard enabled");
                cur_state = 1;
                target_position = current_pose;
                target_position.pose.position.z=TargeyHeight;
                move1.pid_init();
                move1.set_pid_all(0.8,0.2,0.2);
            }
            last_request = ros::Time::now();
        } 
        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    ROS_INFO("step1");
                }
                last_request = ros::Time::now();
            } 
            if(cur_state)
            {
                switch(step)
                {
                    case 1:  // case1 起飞程序
                        if(current_pose.pose.position.z < 0.8) //原为1.5定高，1.2起飞高度
                        {
                            local_pose_pub.publish(target_position);
                        }
                        else
                        {
                            last_time = ros::Time::now();
                            step = 2;
                            target_position = current_pose;
                            target_position.pose.position.x = 0;
                            target_position.pose.position.y = 0;
                            ROS_INFO("step2");
                        }
                        break;
                    case 2:
                        if(ros::Time::now() - last_time < ros::Duration(40.0))
                        {
                            double dt = rate.expectedCycleTime().toSec();
                            angle += dt * angular_speed;
                            if (angle > 2.0*M_PI)
                            {
                                angle -= 2.0*M_PI;
                            }
                            pose.pose.position.x = radius * cos(angle)+2;
                            pose.pose.position.y = radius * sin(angle)+2;
                            local_pose_pub.publish(pose);
                        }
                        else
                        {
                            last_time = ros::Time::now();
                            move1.pid_init();
                            move1.set_pid_all(0.8,0.2,0.2);
                            step = 3;
                        }
                        break;
                    case 3://返航
                        if(flag == 0)
                        {
                            target_position.pose.position.x = 0;
                            target_position.pose.position.y = 0;
                            if(ros::Time::now() - last_time < ros::Duration(6.0))
                            {
                                //local_pose_pub.publish(target_position);
                                move1.fly_to_target(target_position,vel_pub,rate);
                            }
                            else
                            {
                                flag = 1;
                            }
                        }
                        else
                        {
                            flag = 0;
                            last_time = ros::Time::now();
                            ROS_INFO("step18.5");
                            step = 4;
                            target_position.pose.position.z = 0.5;
                        }
                        break;
                    case 4://返航
                        if(ros::Time::now() - last_time < ros::Duration(5.0))
                        {
                            local_pose_pub.publish(target_position);
                        }
                        else
                        {
                            last_time = ros::Time::now();
                            ROS_INFO("step19");
                            step = 5;
                        }
                        break;
                    case 5:
                        if(current_pose.pose.position.z<0.05)
                        {
                            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                            {
                                ROS_INFO("Vehicle disarmed");
                                return 0;
                            }
                        }
                        else
                        {
                            move2.velocity_land(rate);
                        }
                        break;
                }
            }
        }
        rate.sleep();
    }
    return 0;
}

/*------------------分------割------线------------------*/

//比较ab较小的一个数，其中a的值是被比较值，b是限制量（注意，不可用反
double min(double a,double b){
    double ret=0;
    if (a == 0){
        ret=0;
    }else if(fabs(a)>=fabs(b)){
        ret=fabs(b)*a/fabs(a);
    }else{
        ret=fabs(a)*a/fabs(a);
    }
    return ret;
}

//将给定的角度值归一化到 [-π, π] 的范围内
double arc_normalize_yaw(double current_yaw){
    double raw_yaw=current_yaw;
    if(fabs(current_yaw)>3.14){
        raw_yaw = current_yaw > 0 ? current_yaw - 6.28319 : current_yaw + 6.28319;
    }
    return raw_yaw;
}