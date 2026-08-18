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

// 1D Quintic Polynomial for C2 continuous trajectory generation
struct Quintic1D
{
    double c0, c1, c2, c3, c4, c5;

    void compute(double x0, double v0, double a0,
                 double xf, double vf, double af,
                 double T)
    {
        c0 = x0;
        c1 = v0;
        c2 = 0.5 * a0;
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        double h = xf - x0 - v0 * T - 0.5 * a0 * T2;
        double v_diff = vf - v0 - a0 * T;
        double a_diff = af - a0;

        c3 = (10.0 * h - 4.0 * v_diff * T + 0.5 * a_diff * T2) / T3;
        c4 = (-15.0 * h + 7.0 * v_diff * T - a_diff * T2) / T4;
        c5 = (6.0 * h - 3.0 * v_diff * T + 0.5 * a_diff * T2) / T5;
    }

    void eval(double t, double &x, double &v, double &a) const
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        x = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
        v = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
        a = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3;
    }
};

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
    double transition_time_;
    double total_circle_duration_;

    Quintic1D poly_x_;
    Quintic1D poly_y_;
    Quintic1D poly_z_;

public:
    CircleTrajGenerator(ros::NodeHandle &nh) : nh_(nh)
    {
        // Load parameters
        nh_.param<double>("radius", radius_, 1.5);
        nh_.param<double>("linear_vel", linear_vel_, 0.8);
        nh_.param<double>("height", height_, 1.5);
        nh_.param<int>("cycles", cycles_, 15);
        nh_.param<double>("publish_rate", publish_rate_, 50.0);
        nh_.param<int>("k_sample", k_sample_, DEFAULT_KSAMPLE);
        nh_.param<double>("t_step", t_step_, DEFAULT_T_STEP);
        nh_.param<double>("transition_time", transition_time_, 3.0); // 3 seconds smooth transition from center to perimeter
        nh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");

        if (radius_ <= 0.05) radius_ = 0.05;
        omega_ = linear_vel_ / radius_;
        total_circle_duration_ = (cycles_ > 0) ? (transition_time_ + 2.0 * M_PI * cycles_ / omega_) : 1e9;

        mpc_ref_pub_ = nh_.advertise<quadrotor_msgs::mpc_ref_traj>("/mpc_ref_traj", 1);
        debug_ref_path_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/circle_traj/current_ref", 1);

        // 1. 读取初始悬停位置作为圆心 (Center)
        ROS_INFO("[CircleTraj] Waiting for vehicle odometry on %s to set circle center...", odom_topic_.c_str());
        nav_msgs::OdometryConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>(
            odom_topic_, nh_, ros::Duration(5.0));

        if (odom)
        {
            center_x_ = odom->pose.pose.position.x;
            center_y_ = odom->pose.pose.position.y;
            center_z_ = (height_ > 0.3) ? height_ : odom->pose.pose.position.z;
            ROS_INFO("[CircleTraj] Hover position locked as circle center: Center (%.2f, %.2f, %.2f)", center_x_, center_y_, center_z_);
        }
        else
        {
            center_x_ = 0.0;
            center_y_ = 0.0;
            center_z_ = height_;
            ROS_WARN("[CircleTraj] Odometry timeout, using default center: (0.0, 0.0, %.2f)", height_);
        }

        // 2. 初始化从圆心 (center_x, center_y) 到圆周固定切入点 (center_x + R, center_y) 的五次多项式平滑切入曲线
        // 边界条件：
        // 起点 t=0 (圆心): pos=(cx, cy, cz), vel=(0, 0, 0), acc=(0, 0, 0)
        // 终点 t=T_trans (圆周起点): pos=(cx + R, cy, cz), vel=(0, R*omega, 0), acc=(-R*omega^2, 0, 0) (与圆周切向速度和向心加速度严格连续匹配)
        poly_x_.compute(center_x_, 0.0, 0.0,
                        center_x_ + radius_, 0.0, -radius_ * omega_ * omega_,
                        transition_time_);

        poly_y_.compute(center_y_, 0.0, 0.0,
                        center_y_, radius_ * omega_, 0.0,
                        transition_time_);

        poly_z_.compute(center_z_, 0.0, 0.0,
                        center_z_, 0.0, 0.0,
                        transition_time_);

        ROS_INFO("[CircleTraj] Trajectory Config: Center(%.2f, %.2f, %.2f) -> Transition to Perimeter Start(%.2f, %.2f, %.2f) in %.1fs -> Circling (R=%.2fm, V=%.2fm/s, Omega=%.3frad/s, Cycles=%d)",
                 center_x_, center_y_, center_z_, center_x_ + radius_, center_y_, center_z_, transition_time_, radius_, linear_vel_, omega_, cycles_);
    }

    void evaluateTraj(double t_query, Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a)
    {
        if (t_query <= 0.0)
        {
            // 阶段 0：未启动前，保持在初始悬停圆心点
            p << center_x_, center_y_, center_z_;
            v << 0.0, 0.0, 0.0;
            a << 0.0, 0.0, 0.0;
        }
        else if (t_query < transition_time_)
        {
            // 阶段 1：平滑切入阶段 (0s ~ transition_time_): 从圆心飞往固定圆周起点 (cx + R, cy)
            double px, py, pz, vx, vy, vz, ax, ay, az;
            poly_x_.eval(t_query, px, vx, ax);
            poly_y_.eval(t_query, py, vy, ay);
            poly_z_.eval(t_query, pz, vz, az);

            p << px, py, pz;
            v << vx, vy, vz;
            a << ax, ay, az;
        }
        else if (cycles_ > 0 && t_query >= total_circle_duration_)
        {
            // 阶段 3：完成所有圈数后，在终点悬停
            p << center_x_ + radius_, center_y_, center_z_;
            v << 0.0, 0.0, 0.0;
            a << 0.0, 0.0, 0.0;
        }
        else
        {
            // 阶段 2：以 (cx, cy) 为圆心稳定绕圆 (theta 从 0 持续递增)
            double t_circle = t_query - transition_time_;
            double theta = omega_ * t_circle;

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
                evaluateTraj(query_t, p, v, a);

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
