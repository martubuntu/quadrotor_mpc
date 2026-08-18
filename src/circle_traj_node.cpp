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
    double transition_time_;
    double publish_rate_;
    int k_sample_;
    double t_step_;
    std::string odom_topic_;

    double center_x_;
    double center_y_;
    double center_z_;

    // Quintic polynomial coefficients for smooth transition from center to circle entry point
    Eigen::Vector3d p0_, pf_;
    Eigen::Vector3d v0_, vf_;
    Eigen::Vector3d a0_, af_;
    Eigen::Matrix<double, 6, 3> poly_coeffs_; // [c0, c1, c2, c3, c4, c5] for x, y, z

    double total_orbit_duration_;
    double total_mission_duration_;

public:
    CircleTrajGenerator(ros::NodeHandle &nh) : nh_(nh)
    {
        // Load parameters
        nh_.param<double>("radius", radius_, 1.5);
        nh_.param<double>("linear_vel", linear_vel_, 0.8);
        nh_.param<double>("height", height_, 1.5);
        nh_.param<double>("transition_time", transition_time_, 3.5); // Seconds to smoothly fly from center to entry point
        nh_.param<int>("cycles", cycles_, 15);                      // >0: number of cycles, -1: infinite
        nh_.param<double>("publish_rate", publish_rate_, 50.0);
        nh_.param<int>("k_sample", k_sample_, DEFAULT_KSAMPLE);
        nh_.param<double>("t_step", t_step_, DEFAULT_T_STEP);
        nh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");

        bool use_fixed_center;
        double fixed_cx, fixed_cy;
        nh_.param<bool>("use_fixed_center", use_fixed_center, false);
        nh_.param<double>("center_x", fixed_cx, 0.0);
        nh_.param<double>("center_y", fixed_cy, 0.0);

        if (radius_ <= 0.05) radius_ = 0.05;
        if (transition_time_ <= 0.5) transition_time_ = 0.5;

        omega_ = linear_vel_ / radius_;
        total_orbit_duration_ = (cycles_ > 0) ? (2.0 * M_PI * cycles_ / omega_) : 1e9;
        total_mission_duration_ = transition_time_ + total_orbit_duration_;

        mpc_ref_pub_ = nh_.advertise<quadrotor_msgs::mpc_ref_traj>("/mpc_ref_traj", 1);
        debug_ref_path_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/circle_traj/current_ref", 1);

        ROS_INFO("[CircleTraj] Waiting for vehicle odometry on %s...", odom_topic_.c_str());
        nav_msgs::OdometryConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>(
            odom_topic_, nh_, ros::Duration(5.0));

        double x_hover = 0.0, y_hover = 0.0, z_hover = height_;
        if (odom)
        {
            x_hover = odom->pose.pose.position.x;
            y_hover = odom->pose.pose.position.y;
            z_hover = odom->pose.pose.position.z;
            ROS_INFO("[CircleTraj] Received current hover pose: (%.2f, %.2f, %.2f)", x_hover, y_hover, z_hover);
        }
        else
        {
            ROS_WARN("[CircleTraj] No odometry received within 5s, using default (0.0, 0.0, %.2f)", height_);
        }

        // 1. 设置【悬停点为圆心】
        if (use_fixed_center)
        {
            center_x_ = fixed_cx;
            center_y_ = fixed_cy;
            center_z_ = (height_ > 0.3) ? height_ : z_hover;
        }
        else
        {
            center_x_ = x_hover;
            center_y_ = y_hover;
            center_z_ = (height_ > 0.3) ? height_ : z_hover;
        }

        // 2. 规划【从圆心到固定切入点的五次平滑多项式】
        // 初始状态（圆心悬停）：p0 = [cx, cy, cz], v0 = 0, a0 = 0
        p0_ << center_x_, center_y_, center_z_;
        v0_ << 0.0, 0.0, 0.0;
        a0_ << 0.0, 0.0, 0.0;

        // 目标切入点（固定在 +X 轴切入）：pf = [cx + R, cy, cz]
        // 目标切入速度（正切方向 +Y）：vf = [0, R * omega, 0]
        // 目标向心加速度（指向圆心 -X）：af = [-R * omega^2, 0, 0]
        pf_ << center_x_ + radius_, center_y_, center_z_;
        vf_ << 0.0, radius_ * omega_, 0.0;
        af_ << -radius_ * omega_ * omega_, 0.0, 0.0;

        computeQuinticPolynomial();

        ROS_INFO("================================================================================");
        ROS_INFO("[CircleTraj] Initialized: Center LOCKED at Hover Pose (%.2f, %.2f, %.2f)", center_x_, center_y_, center_z_);
        ROS_INFO("[CircleTraj] Fixed Entry Point: (%.2f, %.2f, %.2f) | Radius: %.2fm | Vel: %.2fm/s", pf_.x(), pf_.y(), pf_.z(), radius_, linear_vel_);
        ROS_INFO("[CircleTraj] Smooth Transition: %.1fs (Center -> Entry Point) | Total Orbit: %d cycles (%.1fs)", transition_time_, cycles_, total_orbit_duration_);
        ROS_INFO("================================================================================");
    }

    void computeQuinticPolynomial()
    {
        double T = transition_time_;
        for (int k = 0; k < 3; ++k)
        {
            double dp = pf_[k] - p0_[k];
            double V = vf_[k] * T;
            double A = af_[k] * T * T;

            double c0 = p0_[k];
            double c1 = 0.0;
            double c2 = 0.0;
            double c3 = 10.0 * dp - 4.0 * V + 0.5 * A;
            double c4 = -15.0 * dp + 7.0 * V - 1.0 * A;
            double c5 = 6.0 * dp - 3.0 * V + 0.5 * A;

            poly_coeffs_(0, k) = c0;
            poly_coeffs_(1, k) = c1;
            poly_coeffs_(2, k) = c2;
            poly_coeffs_(3, k) = c3;
            poly_coeffs_(4, k) = c4;
            poly_coeffs_(5, k) = c5;
        }
    }

    void evaluateTrajectory(double t_query, Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a)
    {
        if (t_query < 0.0)
        {
            // 阶段 0：尚未启动，保持在圆心悬停
            p = p0_;
            v = v0_;
            a = a0_;
        }
        else if (t_query < transition_time_)
        {
            // 阶段 1：平滑过渡（从圆心平滑飞到固定切入点，匹配正切速度与向心加速度）
            double tau = t_query / transition_time_;
            double tau2 = tau * tau;
            double tau3 = tau2 * tau;
            double tau4 = tau3 * tau;
            double tau5 = tau4 * tau;
            double T = transition_time_;

            for (int k = 0; k < 3; ++k)
            {
                p[k] = poly_coeffs_(0, k) +
                       poly_coeffs_(1, k) * tau +
                       poly_coeffs_(2, k) * tau2 +
                       poly_coeffs_(3, k) * tau3 +
                       poly_coeffs_(4, k) * tau4 +
                       poly_coeffs_(5, k) * tau5;

                v[k] = (poly_coeffs_(1, k) +
                        2.0 * poly_coeffs_(2, k) * tau +
                        3.0 * poly_coeffs_(3, k) * tau2 +
                        4.0 * poly_coeffs_(4, k) * tau3 +
                        5.0 * poly_coeffs_(5, k) * tau4) / T;

                a[k] = (2.0 * poly_coeffs_(2, k) +
                        6.0 * poly_coeffs_(3, k) * tau +
                        12.0 * poly_coeffs_(4, k) * tau2 +
                        20.0 * poly_coeffs_(5, k) * tau3) / (T * T);
            }
        }
        else if (cycles_ > 0 && t_query >= total_mission_duration_)
        {
            // 阶段 3：完成指定圈数，稳定保持在固定切入点
            p = pf_;
            v << 0.0, 0.0, 0.0;
            a << 0.0, 0.0, 0.0;
        }
        else
        {
            // 阶段 2：以固定圆心平稳绕圆飞行
            double t_orbit = t_query - transition_time_;
            double theta = omega_ * t_orbit;

            p << center_x_ + radius_ * std::cos(theta),
                 center_y_ + radius_ * std::sin(theta),
                 center_z_;

            v << -radius_ * omega_ * std::sin(theta),
                  radius_ * omega_ * std::cos(theta),
                  0.0;

            a << -radius_ * omega_ * omega_ * std::cos(theta),
                 -radius_ * omega_ * omega_ * std::sin(theta),
                  0.0;
        }
    }

    void run()
    {
        ros::Rate rate(publish_rate_);
        ros::Duration(0.5).sleep(); // 确保 subscriber 建立连接
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
                evaluateTrajectory(query_t, p, v, a);

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
