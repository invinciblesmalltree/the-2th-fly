#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lidar_data/LidarPose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <vector>

class target {
  public:
    float x, y, z;
    bool reached = false;

    target(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void fly_to_target(ros::Publisher &local_pos_pub) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        local_pos_pub.publish(pose);
    }
};

float vector2theta(float x, float y) {
    float angle = atan2(y, x);
    return angle < 0 ? angle += 2 * M_PI : angle;
}

mavros_msgs::State current_state;
lidar_data::LidarPose lidar_pose_data;
geometry_msgs::TwistStamped vel_msg;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

void lidar_cb(const lidar_data::LidarPose::ConstPtr &msg) {
    lidar_pose_data = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber lidar_data_sub =
        nh.subscribe<lidar_data::LidarPose>("lidar_data", 10, lidar_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(
        "mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<target> targets = {
        target(0, 0, 1.0),          target(1.41, 0.0, 1.0),
        target(2.82, 0.0, 1.0),     target(3.55, -0.43, 1.0),
        target(3.225, -1.165, 1.0), target(3.225, -1.48, 1.0),
        target(2.82, -2.00, 1.0),   target(1.41, -2.00, 1.0),
        target(0, -2.00, 1.0),      target(0, -2.00, 0.2)};

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    size_t target_index = 0;
    bool ready_to_fly = false;

    vel_msg.twist.linear.y = 0.3;

    while (ros::ok()) {
        if (!current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        } else {
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                    ROS_INFO("Mode: %s", current_state.mode.c_str());
                    ready_to_fly = true;
                }
                last_request = ros::Time::now();
            }
        }

        if (ready_to_fly) {
            if (target_index >= targets.size()) {
                ROS_INFO("All targets reached");
                ros::shutdown();
                break;
            } else if (!targets[target_index].reached) {
                targets[target_index].fly_to_target(local_pos_pub);

                float distance =
                    sqrt(pow(lidar_pose_data.x - targets[target_index].x, 2) +
                         pow(lidar_pose_data.y - targets[target_index].y, 2) +
                         pow(lidar_pose_data.z - targets[target_index].z, 2));
                if (distance < 0.1) {
                    targets[target_index].reached = true;
                    ROS_INFO("Reached target %zu", target_index);
                    target_index++;
                }
            }

            // if (!targets[0].reached) {
            //     targets[0].fly_to_target(local_pos_pub);
            //     float distance = sqrt(pow(lidar_pose_data.x - targets[0].x,
            //     2) +
            //                           pow(lidar_pose_data.y - targets[0].y,
            //                           2) + pow(lidar_pose_data.z -
            //                           targets[0].z, 2));
            //     if (distance < 0.1)
            //         targets[0].reached = true;
            // } else {
            //     float angle_to_target =
            //         vector2theta(3 - lidar_pose_data.x, 4 -
            //         lidar_pose_data.y);
            //     float angular_angle = angle_to_target - lidar_pose_data.yaw;
            //     if (angular_angle > M_PI)
            //         angular_angle -= 2 * M_PI;
            //     if (angular_angle < -M_PI)
            //         angular_angle += 2 * M_PI;
            //     vel_msg.twist.angular.z = angular_angle;

            //     velocity_pub.publish(vel_msg);
            // }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}