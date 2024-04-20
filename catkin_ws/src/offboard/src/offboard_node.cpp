#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    bool reached_first_point = false;
    bool reached_second_point = false;
    bool reached_third_point = false;
    geometry_msgs::PoseStamped pose1, pose2, pose3;

    //点1
    pose1.pose.position.x = 0.0;
    pose1.pose.position.y = 0.0;
    pose1.pose.position.z = 1.0;
    //点2
    pose2.pose.position.x = 2.0;
    pose2.pose.position.y = 0.0;
    pose2.pose.position.z = 1.0;
    //点3
    pose2.pose.position.x = 2.0;
    pose2.pose.position.y = 0.0;
    pose2.pose.position.z = 0.05;

    //状态订阅器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //目标位置发布器
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //定义起飞服务客户端（起飞，降落）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //定义设置模式服务客户端
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
                    ROS_INFO("Mode: %s", current_state.mode.c_str());
                }
                last_request = ros::Time::now();
            }
        }

        //到达第点1，发布点2
        if(reached_first_point && !reached_second_point)
        {
            local_pos_pub.publish(pose2);
        }
        else
        {
            if (ros::Time::now() - last_request > ros::Duration(10.0))
            {
                reached_first_point = true;
                ROS_INFO("Reached first point");
                last_request = ros::Time::now();
            }
        }

        //到达点2，发布点3
        if(reached_second_point)
        {
            local_pos_pub.publish(pose3);
        }
        else
        {
            if (ros::Time::now() - last_request > ros::Duration(10.0))
            {
                reached_second_point = true;
                ROS_INFO("Reached second point");
                last_request = ros::Time::now();
            }
        }

        //到达点3，停止
        if (reached_second_point && ros::Time::now() - last_request > ros::Duration(10.0))
        {
            reached_third_point = true;
            ROS_INFO("Reached third point, turn off");
            last_request = ros::Time::now();
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}