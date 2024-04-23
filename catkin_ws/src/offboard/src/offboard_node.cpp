#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <vector>

class Target {
  public:
    float x, y, z;
    bool reached = false;

    Target(float x, float y, float z) {
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

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<Target> targets = {
        Target(0, 0, 1.0),         Target(2.82, 0.0, 1.0),
        Target(3.55, -0.43, 1.0),  Target(3.225, -1.165, 1.0),
        Target(3.225, -1.48, 1.0), Target(2.82, -2.00, 1.0),
        Target(0, -2.00, 1.0)};

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
                }
                last_request = ros::Time::now();
            }
        }

        if (target_index >= targets.size()) {
            ROS_INFO("All targets reached");
            break;
        }

        if (!targets[target_index].reached) {
            targets[target_index].fly_to_target(local_pos_pub);

            float distance = sqrt(
                pow(current_pose.pose.position.x - targets[target_index].x, 2) +
                pow(current_pose.pose.position.y - targets[target_index].y, 2) +
                pow(current_pose.pose.position.z - targets[target_index].z, 2));
            if (distance < 0.5) {
                targets[target_index].reached = true;
                ROS_INFO("Reached target %zu", target_index);
                target_index++;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}