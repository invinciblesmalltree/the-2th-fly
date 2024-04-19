#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LidarPose
{
public:
    double x = 0; // 机头为X轴正方向
    double y = 0; // 机左为y轴正方向
    double z = 0; // 上方为z轴正方向
    double vx = 0;
    double vy = 0;
    double vz = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0; // 顺时针旋转为正，顺时针转得越多值越大，初始方向为0，范围为0~3.14 （跳变） -3.14~0（归位）

    /**
     * 获得归一化后的yaw
     * 一定大于0，初始方向为0
     * 顺时针方向旋转值为0~6.48
     */
    double getYawSTD()
    {
        double result = 0;
        if (yaw < 0) //如果yaw小于0，则返回2 * 3.141592 + yaw，将负值转换为正值
        {
            return 2 * 3.141592 + yaw;
        }
        else
        {
            return yaw;
        }
    }
};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

LidarPose lidar_pose;
geometry_msgs::PoseStamped instant_position;
void lidar_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    lidar_pose.x = msg->pose.pose.position.x; //单位m
    lidar_pose.y = msg->pose.pose.position.y;
    lidar_pose.z = msg->pose.pose.position.z;
    lidar_pose.vx = msg->twist.twist.linear.x;
    lidar_pose.vy = msg->twist.twist.linear.y;
    lidar_pose.vz = msg->twist.twist.linear.z;

    double quatx = msg->pose.pose.orientation.x;
    double quaty = msg->pose.pose.orientation.y;
    double quatz = msg->pose.pose.orientation.z;
    double quatw = msg->pose.pose.orientation.w;
    tf2::Quaternion q(quatx, quaty, quatz, quatw);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    lidar_pose.roll = roll;
    lidar_pose.pitch = pitch;
    lidar_pose.yaw = -yaw;

    //更新实时位置以发给飞控
    instant_position.pose.position.x = lidar_pose.x;
    instant_position.pose.position.y = lidar_pose.y;
    instant_position.pose.position.z = lidar_pose.z;
    //instant_position.pose.roll = lidar_pose.roll;
    //instant_position.pose.yaw = lidar_pose.yaw;
    //instant_position.pose.pitch = lidar_pose.pitch;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    bool reached_first_point = false;
    bool reached_second_point = false;
    geometry_msgs::PoseStamped pose1, pose2;

    //点1
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0.0;
    pose1.pose.position.z = 0.5;
    //点2
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 0.0;
    pose2.pose.position.z = 0.05;
    //实时位置
    instant_position.pose.position.x = 0.0;
    instant_position.pose.position.y = 0.0;
    instant_position.pose.position.z = 0.0;

    //雷达信息订阅器
    ros::Subscriber lidar_sub = nh.subscribe<nav_msgs::Odometry>
            ("/Odometry", 10, lidar_cb);
    //状态订阅器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //位置目标发布器
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //实时位置发布器
    //ros::Publisher vision_pos_pub=nh.advertise<geometry_msgs::PoseStamped>
            //("mavros/vision_pos/pose", 10);
    //定义起飞服务客户端（起飞，降落）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //定义设置模式服务客户端（设置offboard模式）
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //发布频率
    ros::Rate rate(20.0);

    //等待FCU连接
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //起飞前预发布pose1
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
 
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
        }
        else
        {
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
		    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		    {
			ROS_INFO("Offboard enabled");
            // 打印当前模式
		    }
		    last_request = ros::Time::now();
		}
        }

        if(reached_first_point)
        {
            local_pos_pub.publish(pose2);
        }
        else
        {
            local_pos_pub.publish(pose1);
            if (ros::Time::now() - last_request > ros::Duration(15.0))
            {
                reached_first_point = true;
                ROS_INFO("Reached first point, waiting for 15 seconds before moving to second point.");
                last_request = ros::Time::now();
            }
        }

        if (reached_second_point && ros::Time::now() - last_request > ros::Duration(15.0))
        {
            reached_second_point = true;
            ROS_INFO("Reached second point, turn off");
            last_request = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
