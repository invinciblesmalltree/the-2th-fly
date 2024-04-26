#ifndef CIRCULAR_TRAJECTORY_H  
#define CIRCULAR_TRAJECTORY_H  
  
#include <geometry_msgs/PoseStamped.h>  
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  
  
// 轨迹基类  
class Trajectory {  
public:  
    virtual geometry_msgs::PoseStamped calculate_pose(double time) = 0;  
};  
  
// 圆形轨迹  
class CircularTrajectory : public Trajectory {  
    double radius, center_x, center_y, altitude, angular_velocity;  
  
public:  
    CircularTrajectory(double r, double x, double y, double z, double w)  
        : radius(r), center_x(x), center_y(y), altitude(z), angular_velocity(w) {}  
  
    geometry_msgs::PoseStamped calculate_pose(double time) override {  
        geometry_msgs::PoseStamped pose;  
  
        double theta = angular_velocity * time;  
  
        // 计算位置  
        pose.pose.position.x = center_x + radius * cos(theta);  
        pose.pose.position.y = center_y + radius * sin(theta);  
        pose.pose.position.z = altitude;  
  
        // 计算朝向  
        tf2::Quaternion q;  
        q.setRPY(0, 0, theta); // 设定朝向为绕Z轴旋转theta弧度  
        tf2::convert(q, pose.pose.orientation); // 转换为geometry_msgs中的Quaternion  
  
        return pose;  
    }  
};  
  
#endif // CIRCULAR_TRAJECTORY_H
