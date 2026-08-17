#include <ros/ros.h>
#include <quadrotor_msgs/mpc_ref_point.h>
#include <quadrotor_msgs/mpc_ref_traj.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include <vector>
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
    double center_x_;
    double center_y_;
    double center_z_;
    double radius_;
    double linear_vel_;
    double omega_;
    double start_delay_;
    int cycles_;
    double publish_rate_;
    int k_sample_;
    double t_step_;
    double transition_time_; // Smooth lead-in duration from hover center to circle edge

    double total_circle_duration_;

public:
    CircleTrajGenerator(ros::NodeHandle &nh) : nh_(nh)
    {
        // Load parameters
        nh_.param<double>("center_x", center_x_, 0.0);
        nh_.param<double>("center_y", center_y_, 0.0);
        nh_.param<double>("center_z", center_z_, 1.5);
        nh_.param<double>("radius", radius_, 1.5);
        nh_.param<double>("linear_vel", linear_vel_, 0.8);
        nh_.param<double>("start_delay", start_delay_, 0.0);
        nh_.param<int>("cycles", cycles_, 15); // >0: number of cycles, -1: infinite
        nh_.param<double>("publish_rate", publish_rate_, 50.0);
        nh_.param<int>("k_sample", k_sample_, DEFAULT_KSAMPLE);
        nh_.param<double>("t_step", t_step_, DEFAULT_T_STEP);
        nh_.param<double>("transition_time", transition_time_, 3.0);

        if (radius_ <= 0.05) radius_ = 0.05;
        omega_ = linear_vel_ / radius_;
        total_circle_duration_ = (cycles_ > 0) ? (transition_time_ + 2.0 * M_PI * cycles_ / omega_) : 1e9;

        mpc_ref_pub_ = nh_.advertise<quadrotor_msgs::mpc_ref_traj>("/mpc_ref_traj", 1);
        debug_ref_path_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/circle_traj/current_ref", 1);

        ROS_INFO("[CircleTraj] Ready: Center(%.2f, %.2f, %.2f), Radius: %.2f m, Vel: %.2f m/s, Cycles: %d (Transition: %.1fs)",
                 center_x_, center_y_, center_z_, radius_, linear_vel_, cycles_, transition_time_);
    }

    void evaluateCircle(double t_query, Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a)
    {
        if (t_query < 0.0)
        {
            // Before launch: Hover at center position
            p << center_x_, center_y_, center_z_;
            v << 0.0, 0.0, 0.0;
            a << 0.0, 0.0, 0.0;
        }
        else if (t_query < transition_time_)
        {
            // Smooth cubic transition from hover center (0, 0, z) to circle start (R, 0, z)
            double s = t_query / transition_time_;
            double h = 3.0 * s * s - 2.0 * s * s * s;
            double dh = (6.0 * s - 6.0 * s * s) / transition_time_;
            double ddh = (6.0 - 12.0 * s) / (transition_time_ * transition_time_);

            p << center_x_ + radius_ * h, center_y_, center_z_;
            v << radius_ * dh, 0.0, 0.0;
            a << radius_ * ddh, 0.0, 0.0;
        }
        else if (cycles_ > 0 && t_query >= total_circle_duration_)
        {
            // Completed all cycles: stay at final point
            p << center_x_ + radius_, center_y_, center_z_;
            v << 0.0, 0.0, 0.0;
            a << 0.0, 0.0, 0.0;
        }
        else
        {
            // Active circular flight
            double theta = omega_ * (t_query - transition_time_);
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
        ros::Time node_start = ros::Time::now();

        while (ros::ok())
        {
            ros::Time now = ros::Time::now();
            double elapsed = (now - node_start).toSec();

            // Flight timing
            double t_active = elapsed - start_delay_;

            quadrotor_msgs::mpc_ref_traj traj_msg;

            // Fill MPC horizon (k_sample + 1 points)
            for (int i = 0; i < k_sample_ + 1; ++i)
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

            // Goal definition:
            if (cycles_ > 0 && t_active >= total_circle_duration_)
            {
                // Completed: set goal to stop point so MPC can enter AUTO_HOVER
                traj_msg.goal.x = center_x_ + radius_;
                traj_msg.goal.y = center_y_;
                traj_msg.goal.z = center_z_;
            }
            else
            {
                // In motion: set unreachable goal dummy to prevent early hover exit
                traj_msg.goal.x = 9999.0;
                traj_msg.goal.y = 9999.0;
                traj_msg.goal.z = 9999.0;
            }

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
