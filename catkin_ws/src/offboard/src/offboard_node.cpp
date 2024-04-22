#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class move_to_target
{
    public:
        float x,y,z;
        bool reach = false;

        move_to_target(float x,float y,float z)
        {
            this->x=x;
            this->y=y;
            this->z=z;
        }

        void fly_to_target(ros::Publisher &local_pos_pub) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;

            local_pos_pub.publish(pose);
        }
};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    move_to_target target1(2.82, 0.0, 1.0);
    move_to_target target2(3.55, -0.43, 1.0);
    move_to_target target3(3.225, -1.165, 1.0);
    move_to_target target4(3.225, -1.48, 1.0);
    move_to_target target5(2.82, -2.00, 1.0);
    move_to_target target6(0, -2.00, 1.0);

    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //主循环
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

        //takeoff
        ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/takeoff");
        mavros_msgs::CommandBool srv_takeoff;
        srv_takeoff.request.altitude = 1.0; //高度1米
        if(takeoff_cl.call(srv_takeoff))
            ROS_INFO("takeoff sent %d", srv_takeoff.response.success);

        int flag=1;
        switch(flag)
        {
            case 1:
                target1.fly_to_target(local_pos_pub);
                if (ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    target1.reach = true;
                    ROS_INFO("Reached first point");
                    last_request = ros::Time::now();
                    flag++;
                }
            case 2:
                target2.fly_to_target(local_pos_pub);
                if (ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    target2.reach = true;
                    ROS_INFO("Reached second point");
                    last_request = ros::Time::now();
                    flag++;
                }
            case 3:
                target3.fly_to_target(local_pos_pub);
                if(ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    target3.reach = true;
                    ROS_INFO("Reached third point");
                    last_request = ros::Time::now();
                    flag++;
                }
            case 4:
                target4.fly_to_target(local_pos_pub);
                if (ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    target4.reach = true;
                    ROS_INFO("Reached forth point");
                    last_request = ros::Time::now();
                    flag++;
                }
            case 5:
                target5.fly_to_target(local_pos_pub);
                if (ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    target5.reach = true;
                    ROS_INFO("Reached fifth point");
                    last_request = ros::Time::now();
                    flag++;
                }
            case 6:
                target6.fly_to_target(local_pos_pub);
                if (ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    target6.reach = true;
                    ROS_INFO("Reached sixth point");
                    last_request = ros::Time::now();
                    flag++;
                }
        }

        //land
        ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/land");
        mavros_msgs::CommandBool srv_land;
        if (land_client.call(srv_land) && srv_land.response.success)
                ROS_INFO("land sent %d", srv_land.response.success);

        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}