#include <cmath>
#include <cv_detect/BarMsg.h>
#include <cv_detect/LedMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lidar_data/LidarPose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <vector>

class target {
  public:
    float x, y, z, yaw;
    bool reached = false;

    target(float x, float y, float z, float yaw) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->yaw = yaw;
    }

    void fly_to_target(ros::Publisher &local_pos_pub) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = sin(yaw / 2);
        pose.pose.orientation.w = cos(yaw / 2);

        local_pos_pub.publish(pose);
    }
};

float vector2theta(float x, float y) {
    float angle = atan2(y, x);
    return angle < 0 ? angle += 2 * M_PI : angle;
}

mavros_msgs::State current_state;
lidar_data::LidarPose lidar_pose_data;
cv_detect::LedMsg blue_data;
cv_detect::BarMsg barcode_data;
std_msgs::Int32 supersonic_data;
geometry_msgs::TwistStamped vel_msg;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

void lidar_cb(const lidar_data::LidarPose::ConstPtr &msg) {
    lidar_pose_data = *msg;
}

void led_cb(const cv_detect::LedMsg::ConstPtr &msg) { blue_data = *msg; }

void barcode_cb(const cv_detect::BarMsg::ConstPtr &msg) { barcode_data = *msg; }

void supersonic_cb(const std_msgs::Int32::ConstPtr &msg) {
    supersonic_data = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber lidar_data_sub =
        nh.subscribe<lidar_data::LidarPose>("lidar_data", 10, lidar_cb);
    ros::Subscriber led_sub =
        nh.subscribe<cv_detect::LedMsg>("bule_msg", 10, led_cb);
    ros::Subscriber barcode_sub =
        nh.subscribe<cv_detect::BarMsg>("barcode_msg", 10, barcode_cb);
    ros::Subscriber supersonic_sub =
        nh.subscribe<std_msgs::Int32>("supersonic_data", 10, supersonic_cb);
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
        target(0, 0, 1.5, 0.0),
        target(0.71, 0.0, 1.5, 0.0),
        target(1.41, 0.0, 1.5, 0.0),
        target(2.12, 0.0, 1.5, 0.0),
        target(2.82, 0.0, 1.5, 0.0),
        target(3.185, -0.215, 1.5, -M_PI / 6),
        target(3.35, -0.43, 1.5, -M_PI / 3), // x少了0.2
        target(3.388, -0.798, 1.5, -2 * M_PI / 3),
        target(3.225, -1.165, 1.5, -2 * M_PI / 3),
        target(3.225, -1.48, 1.5, -7 * M_PI / 12),
        target(3.023, -1.74, 1.5, -2 * M_PI / 3),
        target(2.82, -1.80, 1.5, -5 * M_PI / 6), // y多了0.2
        target(2.12, -2.00, 1.5, -M_PI),
        target(1.41, -2.00, 1.5, -M_PI),
        target(0.71, -2.00, 1.5, -M_PI),
        target(0, -2.00, 1.5, -M_PI),
        target(0, 0, 0.8, -M_PI),
        target(0, 0, 0.1, -M_PI)};

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
    int mode = 0;
    int n = 0;
    bool has_passed_blue = false;
    bool has_passed_half = false;
    target blue_point(0, 0, 0, 0);
    target scan_point(0.6, -0.8, 1.5, -M_PI);
    target pole_point(0, -1.0, 1.5, -M_PI);
    target round_point(0, -1.0, 1.5, -M_PI);

    while (ros::ok) {
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
                    break;
                }
                last_request = ros::Time::now();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        if (mode == 0) { // 正常巡线代码
            if (target_index >= targets.size()) {
                ROS_INFO("All targets reached");
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
                ROS_INFO("Vehicle disarmed");
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
                if (!has_passed_blue && blue_data.value) {
                    blue_point.x = lidar_pose_data.x;
                    blue_point.y = lidar_pose_data.y;
                    blue_point.z = 1.5;
                    blue_point.yaw = lidar_pose_data.yaw;
                    last_request = ros::Time::now();
                    mode = 1;
                    ROS_INFO("Blue object detected");
                    ROS_INFO("Mode 1");
                }
            }
        } else if (mode == 1) { // 蓝色物体检测并悬停代码
            blue_point.fly_to_target(local_pos_pub);
            if (ros::Time::now() - last_request > ros::Duration(5.0)) {
                has_passed_blue = true;
                mode = 2;
                ROS_INFO("Mode 2");
            }
        } else if (mode == 2) { // 找杆代码
            if (!scan_point.reached) {
                scan_point.fly_to_target(local_pos_pub);
                float distance = sqrt(pow(lidar_pose_data.x - scan_point.x, 2) +
                                      pow(lidar_pose_data.y - scan_point.y, 2) +
                                      pow(lidar_pose_data.z - scan_point.z, 2));
                if (distance < 0.1)
                    scan_point.reached = true;
            } else {
                vel_msg.twist.linear.x = 0;
                vel_msg.twist.linear.y = -0.2;
                vel_msg.twist.linear.z = 0;
                velocity_pub.publish(vel_msg);
                ROS_INFO("Supersonic data: %dcm", supersonic_data.data);

                if (supersonic_data.data < 100) {
                    pole_point.x = lidar_pose_data.x -
                                   supersonic_data.data / 100.0 - 0.225;
                    pole_point.y = lidar_pose_data.y - 0.1;
                    pole_point.z = 1.5;
                    pole_point.yaw = lidar_pose_data.yaw;
                    scan_point.x = pole_point.x + 0.3;
                    scan_point.y = pole_point.y;
                    scan_point.z = 1.5;
                    scan_point.reached = false;
                    round_point.x = pole_point.x + 0.5;
                    round_point.y = pole_point.y;
                    round_point.z = 1.5;
                    round_point.yaw = -M_PI;
                    mode = 3;
                    ROS_INFO("Pole: (%f, %f)", pole_point.x, pole_point.y);
                    ROS_INFO("Mode 3");
                }
            }
        } else if (mode == 3) { // 扫码代码
            if (!scan_point.reached) {
                scan_point.fly_to_target(local_pos_pub);
                float distance = sqrt(pow(lidar_pose_data.x - scan_point.x, 2) +
                                      pow(lidar_pose_data.y - scan_point.y, 2) +
                                      pow(lidar_pose_data.z - scan_point.z, 2));
                if (distance < 0.1)
                    scan_point.reached = true;
            } else {
                if (barcode_data.n != -1) {
                    n = barcode_data.n;
                    targets.pop_back();
                    targets.pop_back();
                    targets.pop_back();
                    targets.pop_back();
                    targets.push_back(target(0, -2.0 + n * 0.1, 1.5, 0.0));
                    targets.push_back(target(0, -2.0 + n * 0.1, 0.8, 0.0));
                    targets.push_back(target(0, -2.0 + n * 0.1, 0.1, 0.0));
                    mode = 4;
                    ROS_INFO("Barcode: %d", n);
                    ROS_INFO("Mode 4");
                }
            }
        } else if (mode == 4) { // 绕杆代码
            if (!round_point.reached) {
                round_point.fly_to_target(local_pos_pub);
                float distance =
                    sqrt(pow(lidar_pose_data.x - round_point.x, 2) +
                         pow(lidar_pose_data.y - round_point.y, 2) +
                         pow(lidar_pose_data.z - round_point.z, 2));
                if (distance < 0.1)
                    round_point.reached = true;
            } else {
                if (lidar_pose_data.yaw > 11 * M_PI / 6) {
                    has_passed_half = true;
                    ROS_INFO("Half passed");
                }
                if (has_passed_half && lidar_pose_data.yaw < M_PI) {
                    mode = 5;
                    ROS_INFO("Full passed");
                    ROS_INFO("Mode 5");
                }
                float angle_to_target =
                    vector2theta(pole_point.x - lidar_pose_data.x,
                                 pole_point.y - lidar_pose_data.y);
                float angular_angle = angle_to_target - lidar_pose_data.yaw;
                if (angular_angle > M_PI)
                    angular_angle -= 2 * M_PI;
                if (angular_angle < -M_PI)
                    angular_angle += 2 * M_PI;
                vel_msg.twist.angular.z = angular_angle * 2.5;
                float distance = sqrt(pow(lidar_pose_data.x - pole_point.x, 2) +
                                      pow(lidar_pose_data.y - pole_point.y, 2));
                float v_x = 0.2 * cos(lidar_pose_data.yaw + M_PI / 2),
                      v_y = 0.2 * sin(lidar_pose_data.yaw + M_PI / 2),
                      delta_dis = distance - 0.5;
                v_x += delta_dis * 2 * cos(angle_to_target);
                v_y += delta_dis * 2 * sin(angle_to_target);
                ROS_INFO("%f %f %f", delta_dis, v_x, v_y);
                vel_msg.twist.linear.x = v_x;
                vel_msg.twist.linear.y = v_y;
                vel_msg.twist.linear.z = 1.5 - lidar_pose_data.z;
                velocity_pub.publish(vel_msg);
            }
        } else if (mode == 5) { // 返回蓝色物体代码
            if (!blue_point.reached) {
                blue_point.fly_to_target(local_pos_pub);
                float distance = sqrt(pow(lidar_pose_data.x - blue_point.x, 2) +
                                      pow(lidar_pose_data.y - blue_point.y, 2) +
                                      pow(lidar_pose_data.z - blue_point.z, 2));
                if (distance < 0.1)
                    blue_point.reached = true;
            } else {
                mode = 0;
                ROS_INFO("Mode 0");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}