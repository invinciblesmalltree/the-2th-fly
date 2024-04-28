#include <geometry_msgs/PoseStamped.h>
#include <lidar_data/LidarPose.h>
#include <ros/ros.h>
#include <tf/tf.h>

class PoseDataNode {
  public:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber pose_sub;

    PoseDataNode() {
        // 初始化发布者，发布 LidarPose 消息
        pose_pub = nh.advertise<lidar_data::LidarPose>("lidar_data", 10);
        ROS_INFO("LidarPose Pub");
        // 初始化订阅者，订阅 PoseStamped 消息
        ROS_INFO("LidarPose Sub");
        pose_sub = nh.subscribe("mavros/local_position/pose", 10,
                                &PoseDataNode::poseCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        // 从 PoseStamped 消息中获取位置和姿态（四元数）
        const auto &pose = msg->pose;

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
        pose_pub.publish(output);
        ROS_INFO("Position=(%.2f, %.2f, %.2f), "
                 "Orientation=(%.2f, %.2f, %.2f) rad",
                 x, y, z, roll, pitch, yaw);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_stamped_node");
    PoseDataNode node;

    ros::spin();
    return 0;
}
