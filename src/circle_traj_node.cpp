#include <ros/ros.h>
#include <ros/topic.h>
#include <quadrotor_msgs/mpc_ref_point.h>
#include <quadrotor_msgs/mpc_ref_traj.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Eigen>

#define DEFAULT_KSAMPLE 20
#define DEFAULT_T_STEP 0.1

class CircleTrajGenerator
{
private:
    ros::NodeHandle nh_;
    ros::Publisher mpc_ref_pub_;
    ros::Publisher debug_ref_path_pub_;

    // Circle parameters
    double radius_;
    double linear_vel_;
    double height_;
    double omega_;
    int cycles_;
    double publish_rate_;
    int k_sample_;
    double t_step_;
    std::string odom_topic_;

    double center_x_;
    double center_y_;
    double center_z_;
    double total_circle_duration_;

public:
    CircleTrajGenerator(ros::NodeHandle &nh) : nh_(nh)
    {
        // Load parameters
        nh_.param<double>("radius", radius_, 1.5);
        nh_.param<double>("linear_vel", linear_vel_, 0.8);
        nh_.param<double>("height", height_, 1.5);
        nh_.param<int>("cycles", cycles_, 15); // >0: number of cycles, -1: infinite
        nh_.param<double>("publish_rate", publish_rate_, 50.0);
        nh_.param<int>("k_sample", k_sample_, DEFAULT_KSAMPLE);
        nh_.param<double>("t_step", t_step_, DEFAULT_T_STEP);
        nh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");

        if (radius_ <= 0.05) radius_ = 0.05;
        omega_ = linear_vel_ / radius_;
        total_circle_duration_ = (cycles_ > 0) ? (2.0 * M_PI * cycles_ / omega_) : 1e9;

        mpc_ref_pub_ = nh_.advertise<quadrotor_msgs::mpc_ref_traj>("/mpc_ref_traj", 1);
        debug_ref_path_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/circle_traj/current_ref", 1);

        ROS_INFO("[CircleTraj] Waiting for vehicle odometry on %s...", odom_topic_.c_str());
        nav_msgs::OdometryConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>(
            odom_topic_, nh_, ros::Duration(5.0));

        double x0 = 0.0, y0 = 0.0, z0 = height_;
        if (odom)
        {
            x0 = odom->pose.pose.position.x;
            y0 = odom->pose.pose.position.y;
            z0 = odom->pose.pose.position.z;
            ROS_INFO("[CircleTraj] Received current hover pose: (%.2f, %.2f, %.2f)", x0, y0, z0);
        }
        else
        {
            ROS_WARN("[CircleTraj] No odometry received within 5s, using default start (0.0, 0.0, %.2f)", height_);
        }

        // 以当前悬停位置作为起点，圆心设置在 +Y 方向
        center_x_ = x0;
        center_y_ = y0 + radius_;
        center_z_ = (height_ > 0.3) ? height_ : z0;

        ROS_INFO("[CircleTraj] Ready: Center(%.2f, %.2f, %.2f), Start(%.2f, %.2f, %.2f), R=%.2fm, V=%.2fm/s, Omega=%.3frad/s, Cycles=%d",
                 center_x_, center_y_, center_z_, x0, y0, center_z_, radius_, linear_vel_, omega_, cycles_);
    }

    void evaluateCircle(double t_query, Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a)
    {
        if (t_query < 0.0)
        {
            // Before launch: stay at start hover point
            p << center_x_, center_y_ - radius_, center_z_;
            v << 0.0, 0.0, 0.0;
            a << 0.0, 0.0, 0.0;
        }
        else if (cycles_ > 0 && t_query >= total_circle_duration_)
        {
            // Completed all cycles: stay at final point
            p << center_x_, center_y_ - radius_, center_z_;
            v << 0.0, 0.0, 0.0;
            a << 0.0, 0.0, 0.0;
        }
        else
        {
            // Active circular flight
            double theta = omega_ * t_query;
            p << center_x_ + radius_ * std::sin(theta),
                 center_y_ - radius_ * std::cos(theta),
                 center_z_;

            v << radius_ * omega_ * std::cos(theta),
                 radius_ * omega_ * std::sin(theta),
                 0.0;

            a << -radius_ * omega_ * omega_ * std::sin(theta),
                  radius_ * omega_ * omega_ * std::cos(theta),
                  0.0;
        }
    }

    void run()
    {
        ros::Rate rate(publish_rate_);
        // 短暂延时确保 subscriber 建立连接
        ros::Duration(0.5).sleep();
        ros::Time node_start = ros::Time::now();

        while (ros::ok())
        {
            ros::Time now = ros::Time::now();
            double t_active = (now - node_start).toSec();

            quadrotor_msgs::mpc_ref_traj traj_msg;
            traj_msg.mpc_ref_points.reserve(k_sample_ + 1);

            // Fill MPC horizon (k_sample + 1 points)
            for (int i = 0; i <= k_sample_; ++i)
            {
                double query_t = t_active + i * t_step_;
                Eigen::Vector3d p, v, a;
                evaluateCircle(query_t, p, v, a);

                quadrotor_msgs::mpc_ref_point point_msg;
                point_msg.position.x = p[0];
                point_msg.position.y = p[1];
                point_msg.position.z = p[2];
                point_msg.velocity.x = v[0];
                point_msg.velocity.y = v[1];
                point_msg.velocity.z = v[2];
                point_msg.acceleration.x = a[0];
                point_msg.acceleration.y = a[1];
                point_msg.acceleration.z = a[2];

                traj_msg.mpc_ref_points.push_back(point_msg);
            }

            // Goal definition: 将 goal 设为圆心，防止巡航过程中误触 reachgoal()
            traj_msg.goal.x = center_x_;
            traj_msg.goal.y = center_y_;
            traj_msg.goal.z = center_z_;

            mpc_ref_pub_.publish(traj_msg);

            // Publish visual debug pose
            if (!traj_msg.mpc_ref_points.empty())
            {
                geometry_msgs::PoseStamped debug_pose;
                debug_pose.header.stamp = now;
                debug_pose.header.frame_id = "world";
                debug_pose.pose.position.x = traj_msg.mpc_ref_points[0].position.x;
                debug_pose.pose.position.y = traj_msg.mpc_ref_points[0].position.y;
                debug_pose.pose.position.z = traj_msg.mpc_ref_points[0].position.z;
                debug_pose.pose.orientation.w = 1.0;
                debug_ref_path_pub_.publish(debug_pose);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_traj_node");
    ros::NodeHandle nh("~");

    CircleTrajGenerator generator(nh);
    generator.run();

    return 0;
}
