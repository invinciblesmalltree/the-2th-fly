#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class move_to_target
{
        float x,y,z;
        bool reach = false;

        move_to_target(float x,float y,float z)
        {
            this->x=x;
            this->y=y;
            this->z=z;
        }

    public:
        void fly_to_target(ros::Publisher &local_pos_pub) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;

            local_pos_pub.publish(pose);
        }
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

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

    move_to_target target1(0.0, 0.0, 1.0);
    move_to_target target2(2.0, 0.0, 1.0);
    move_to_target target3(2.0, 0.0, 0.05);

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

        if(!target1.reach && !target2.reach && !target3.reach)
        {
            target1.fly_to_target(local_pos_pub);
            if (ros::Time::now() - last_request > ros::Duration(10.0))
            {
                target1.reach = true;
                ROS_INFO("Reached first point");
                last_request = ros::Time::now();
            }
        }
        else if(target1.reach && !target2.reach && !target3.reach)
        {
            target2.fly_to_target(local_pos_pub);
            if (ros::Time::now() - last_request > ros::Duration(10.0))
            {
                reached_second_point = true;
                ROS_INFO("Reached second point");
                last_request = ros::Time::now();
            }
        }
        else if(target1.reach && target2.reach && !target3.reach)
        {
            target3.fly_to_target(local_pos_pub);
            if(ros::Time::now() - last_request > ros::Duration(10.0))
            {
                target3.reach = true;
                ROS_INFO("Reached third point, turn off");
                last_request = ros::Time::now();
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}