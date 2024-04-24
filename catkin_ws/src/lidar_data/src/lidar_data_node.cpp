#include <lidar_data/LidarPose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

class LidarDataNode {
  public:
    ros::NodeHandle nh;
    ros::Publisher data_pub;
    ros::Subscriber odom_sub;

    LidarDataNode() {
        // 初始化发布者，发布 LidarPose 消息
        data_pub = nh.advertise<lidar_data::LidarPose>("lidar_data", 10);
        // 初始化订阅者，订阅 Odometry 消息
        odom_sub =
            nh.subscribe("/Odometry", 10, &LidarDataNode::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        // 从 Odometry 消息中获取位置和姿态（四元数）
        const auto &pose = msg->pose.pose;

        // 提取位置坐标
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;

        // 提取并转换四元数为欧拉角
        tf::Quaternion q(pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (roll < 0)
            roll += 2 * M_PI;
        if (pitch < 0)
            pitch += 2 * M_PI;
        if (yaw < 0)
            yaw += 2 * M_PI;

        // 创建并填充 LidarPose 消息
        lidar_data::LidarPose output;
        output.x = x;
        output.y = y;
        output.z = z;
        output.roll = roll;
        output.pitch = pitch;
        output.yaw = yaw;

        // 发布消息
        data_pub.publish(output);
        ROS_INFO("Position=(%.2f, %.2f, %.2f), "
                 "Orientation=(%.2f, %.2f, %.2f) rad",
                 x, y, z, roll, pitch, yaw);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_data_node");
    LidarDataNode node;
    ros::spin();
    return 0;
}
