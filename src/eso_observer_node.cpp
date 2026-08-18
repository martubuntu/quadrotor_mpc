#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <string>

class ESOObserver
{
public:
    ESOObserver()
        : nh_(),
          pnh_("~")
    {
        pnh_.param("observer_bw", observer_bw_, 4.0);
        pnh_.param("disturbance_limit", disturbance_limit_, 5.0);

        beta1_ = 3.0 * observer_bw_;
        beta2_ = 3.0 * observer_bw_ * observer_bw_;
        beta3_ = observer_bw_ * observer_bw_ * observer_bw_;

        odom_sub_ = nh_.subscribe(
            "/mavros/local_position/odom",
            20,
            &ESOObserver::odomCallback,
            this);

        cmd_sub_ = nh_.subscribe(
            "/mavros/setpoint_raw/attitude",
            20,
            &ESOObserver::cmdCallback,
            this);

        hover_sub_ = nh_.subscribe(
            "/mpc_debug/hover_thrust",
            20,
            &ESOObserver::hoverCallback,
            this);

        state_sub_ = nh_.subscribe(
            "/mavros/state",
            20,
            &ESOObserver::stateCallback,
            this);

        mode_sub_ = nh_.subscribe(
            "/mpc_debug/mode",
            20,
            &ESOObserver::modeCallback,
            this);

        disturbance_pub_ =
            nh_.advertise<geometry_msgs::Vector3Stamped>(
                "/eso/disturbance", 10);

        velocity_pub_ =
            nh_.advertise<geometry_msgs::Vector3Stamped>(
                "/eso/estimated_velocity", 10);

        z1_.setZero();
        z2_.setZero();
        z3_.setZero();

        initialized_ = false;

        thrust_cmd_ = 0.0;
        hover_thrust_ = 0.58; // Iris simulation default

        mpc_mode_ = 0;

        ROS_INFO("[ESO] Observer initialized. Bandwidth = %.2f rad/s (b1=%.2f, b2=%.2f, b3=%.2f), Limit = %.1f m/s^2",
                 observer_bw_, beta1_, beta2_, beta3_, disturbance_limit_);
    }

private:
    bool observerActive() const
    {
        // Active when ARMED, in OFFBOARD mode, and in HOVER(1) or TRACKING(2)
        return current_state_.armed &&
               current_state_.mode == "OFFBOARD" &&
               (mpc_mode_ == 1 || mpc_mode_ == 2);
    }

    void resetObserver(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const ros::Time& stamp)
    {
        z1_ = position;
        z2_ = velocity;
        z3_.setZero();

        last_stamp_ = stamp;
        initialized_ = true;

        ROS_INFO("[ESO] Observer reset at pos(%.2f, %.2f, %.2f)", position.x(), position.y(), position.z());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        Eigen::Vector3d position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        if (q.norm() < 1e-6) return;
        q.normalize();

        // Convert body twist to world frame
        Eigen::Vector3d v_body(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);

        Eigen::Vector3d v_world = q * v_body;

        if (!observerActive())
        {
            initialized_ = false;
            publishZero(msg->header.stamp);
            return;
        }

        if (!initialized_)
        {
            resetObserver(position, v_world, msg->header.stamp);
            return;
        }

        double dt = (msg->header.stamp - last_stamp_).toSec();
        last_stamp_ = msg->header.stamp;

        if (dt <= 0.0 || dt > 0.1)
        {
            resetObserver(position, v_world, msg->header.stamp);
            return;
        }

        // Compute nominal collective thrust acceleration
        double hover = std::max(hover_thrust_, 0.1);
        double aT_eff = thrust_cmd_ * 9.8066 / hover;

        // Nominal acceleration vector in world frame
        Eigen::Vector3d body_z_world = q.toRotationMatrix().col(2);
        Eigen::Vector3d a_nom = aT_eff * body_z_world;
        a_nom.z() -= 9.8066;

        // Linear Extended State Observer (ESO)
        Eigen::Vector3d e = position - z1_;

        Eigen::Vector3d z1_dot = z2_ + beta1_ * e;
        Eigen::Vector3d z2_dot = a_nom + z3_ + beta2_ * e;
        Eigen::Vector3d z3_dot = beta3_ * e;

        z1_ += z1_dot * dt;
        z2_ += z2_dot * dt;
        z3_ += z3_dot * dt;

        // Disturbance clamping for safety
        for (int i = 0; i < 3; ++i)
        {
            z3_[i] = std::max(-disturbance_limit_, std::min(disturbance_limit_, z3_[i]));
        }

        publishEstimate(msg->header.stamp);
    }

    void publishEstimate(const ros::Time& stamp)
    {
        geometry_msgs::Vector3Stamped d_msg;
        d_msg.header.stamp = stamp;
        d_msg.header.frame_id = "world";
        d_msg.vector.x = z3_.x();
        d_msg.vector.y = z3_.y();
        d_msg.vector.z = z3_.z();
        disturbance_pub_.publish(d_msg);

        geometry_msgs::Vector3Stamped v_msg;
        v_msg.header = d_msg.header;
        v_msg.vector.x = z2_.x();
        v_msg.vector.y = z2_.y();
        v_msg.vector.z = z2_.z();
        velocity_pub_.publish(v_msg);
    }

    void publishZero(const ros::Time& stamp)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "world";
        msg.vector.x = 0.0;
        msg.vector.y = 0.0;
        msg.vector.z = 0.0;
        disturbance_pub_.publish(msg);
    }

    void cmdCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
    {
        thrust_cmd_ = msg->thrust;
    }

    void hoverCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        if (msg->data > 0.1 && msg->data < 0.9)
        {
            hover_thrust_ = msg->data;
        }
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state_ = *msg;
    }

    void modeCallback(const std_msgs::Int8::ConstPtr& msg)
    {
        mpc_mode_ = msg->data;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber hover_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber mode_sub_;

    ros::Publisher disturbance_pub_;
    ros::Publisher velocity_pub_;

    mavros_msgs::State current_state_;
    int mpc_mode_;

    double observer_bw_;
    double disturbance_limit_;

    double beta1_;
    double beta2_;
    double beta3_;

    double thrust_cmd_;
    double hover_thrust_;

    Eigen::Vector3d z1_;  // estimated position
    Eigen::Vector3d z2_;  // estimated velocity
    Eigen::Vector3d z3_;  // estimated disturbance acceleration

    bool initialized_;
    ros::Time last_stamp_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "eso_observer_node");
    ESOObserver observer;
    ros::spin();
    return 0;
}
