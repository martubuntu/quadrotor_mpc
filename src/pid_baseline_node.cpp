#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/mpc_ref_point.h>
#include <quadrotor_msgs/mpc_ref_traj.h>
#include <Eigen/Eigen>

enum PIDMode
{
    PID_TAKEOFF = 0,
    PID_HOVER = 1,
    PID_TRACKING = 2
};

class PIDBaselineController
{
private:
    ros::NodeHandle nh_;

    // Parameters
    double takeoff_height_;
    double ctrl_hz_;
    std::string odom_topic_;
    bool auto_arm_and_offboard_;

    // ROS Interface
    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber traj_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher setpoint_raw_pub_;
    ros::Publisher debug_mode_pub_;
    ros::Publisher debug_ref_pose_pub_;
    ros::Publisher debug_control_pub_;
    ros::Publisher debug_hover_thrust_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    // State Variables
    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_odom_;
    nav_msgs::Odometry start_odom_;
    nav_msgs::Odometry hover_odom_;
    quadrotor_msgs::mpc_ref_traj traj_msg_;

    PIDMode mode_;
    bool has_odom_;
    bool is_initialized_;
    bool traj_received_;
    ros::Time last_request_time_;
    Eigen::Vector3d goal_;

public:
    PIDBaselineController(ros::NodeHandle &nh) : nh_(nh), mode_(PID_TAKEOFF),
                                                 has_odom_(false), is_initialized_(false),
                                                 traj_received_(false)
    {
        nh_.param<double>("/takeoff_height", takeoff_height_, 1.5);
        nh_.param<double>("/ctrl_hz", ctrl_hz_, 50.0);
        nh_.param<std::string>("/odomTopicName", odom_topic_, "/mavros/local_position/odom");
        nh_.param<bool>("/auto_arm_and_offboard", auto_arm_and_offboard_, true);

        // Subscribers
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &PIDBaselineController::odomCallback, this);
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &PIDBaselineController::stateCallback, this);
        traj_sub_ = nh_.subscribe<quadrotor_msgs::mpc_ref_traj>("/mpc_ref_traj", 1, &PIDBaselineController::trajCallback, this);
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &PIDBaselineController::imuCallback, this);

        // Publishers
        setpoint_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
        debug_mode_pub_ = nh_.advertise<std_msgs::Int8>("/mpc_debug/mode", 1);
        debug_ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mpc_debug/ref_pose", 1);
        debug_control_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/mpc_debug/raw_control", 1);
        debug_hover_thrust_pub_ = nh_.advertise<std_msgs::Float64>("/mpc_debug/hover_thrust", 1);

        // Service Clients
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        last_request_time_ = ros::Time::now();
        ROS_INFO("[PID Baseline] Initialized. Target Takeoff Height: %.2f m, Rate: %.1f Hz", takeoff_height_, ctrl_hz_);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        current_odom_ = *msg;
        if (!has_odom_)
        {
            start_odom_ = *msg;
            hover_odom_ = *msg;
            hover_odom_.pose.pose.position.z = takeoff_height_;
            has_odom_ = true;
            ROS_INFO("[PID Baseline] Odometry received at (%.2f, %.2f, %.2f)",
                     msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        }
    }

    void stateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state_ = *msg;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // IMU callback for telemetry
    }

    void trajCallback(const quadrotor_msgs::mpc_ref_traj::ConstPtr &msg)
    {
        traj_msg_ = *msg;
        goal_ << msg->goal.x, msg->goal.y, msg->goal.z;

        if (mode_ == PID_HOVER && !reachGoal(current_odom_, goal_))
        {
            mode_ = PID_TRACKING;
            traj_received_ = true;
            ROS_INFO("[PID Baseline] Circle Trajectory Received -> Switched to PID_TRACKING");
        }
    }

    bool reachGoal(const nav_msgs::Odometry &odom, const Eigen::Vector3d &goal)
    {
        double dx = odom.pose.pose.position.x - goal[0];
        double dy = odom.pose.pose.position.y - goal[1];
        double dz = odom.pose.pose.position.z - goal[2];
        return (dx * dx + dy * dy + dz * dz) < 0.08;
    }

    void run()
    {
        ros::Rate rate(ctrl_hz_);
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        // Pre-stream setpoints before requesting OFFBOARD
        while (ros::ok() && !current_state_.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("[PID Baseline] Connected to FCU. Starting control loop...");

        while (ros::ok())
        {
            ros::spinOnce();

            if (!has_odom_)
            {
                rate.sleep();
                continue;
            }

            ros::Time now = ros::Time::now();

            // Handle Auto Arm & Offboard
            if (auto_arm_and_offboard_)
            {
                if (current_state_.mode != "OFFBOARD" && (now - last_request_time_ > ros::Duration(2.0)))
                {
                    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    {
                        ROS_INFO("[PID Baseline] OFFBOARD enabled");
                    }
                    last_request_time_ = now;
                }
                else if (!current_state_.armed && (now - last_request_time_ > ros::Duration(2.0)))
                {
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("[PID Baseline] Vehicle armed");
                    }
                    last_request_time_ = now;
                }
            }

            // State Machine
            mavros_msgs::PositionTarget setpoint;
            setpoint.header.stamp = now;
            setpoint.header.frame_id = "world";
            setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

            geometry_msgs::PoseStamped ref_pose;
            ref_pose.header.stamp = now;
            ref_pose.header.frame_id = "world";

            switch (mode_)
            {
            case PID_TAKEOFF:
            {
                // Takeoff target: position (start_x, start_y, takeoff_height)
                setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                setpoint.position.x = start_odom_.pose.pose.position.x;
                setpoint.position.y = start_odom_.pose.pose.position.y;
                setpoint.position.z = takeoff_height_;
                setpoint.velocity.x = 0.0;
                setpoint.velocity.y = 0.0;
                setpoint.velocity.z = 0.0;
                setpoint.yaw = 0.0;

                ref_pose.pose.position.x = setpoint.position.x;
                ref_pose.pose.position.y = setpoint.position.y;
                ref_pose.pose.position.z = setpoint.position.z;
                ref_pose.pose.orientation.w = 1.0;

                Eigen::Vector3d takeoff_goal(start_odom_.pose.pose.position.x,
                                             start_odom_.pose.pose.position.y,
                                             takeoff_height_);
                if (reachGoal(current_odom_, takeoff_goal))
                {
                    mode_ = PID_HOVER;
                    hover_odom_ = current_odom_;
                    ROS_INFO("[PID Baseline] Takeoff complete -> Switched to PID_HOVER");
                }
                break;
            }

            case PID_HOVER:
            {
                // Hover at hover_odom_
                setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                setpoint.position.x = hover_odom_.pose.pose.position.x;
                setpoint.position.y = hover_odom_.pose.pose.position.y;
                setpoint.position.z = hover_odom_.pose.pose.position.z;
                setpoint.velocity.x = 0.0;
                setpoint.velocity.y = 0.0;
                setpoint.velocity.z = 0.0;
                setpoint.yaw = 0.0;

                ref_pose.pose.position.x = setpoint.position.x;
                ref_pose.pose.position.y = setpoint.position.y;
                ref_pose.pose.position.z = setpoint.position.z;
                ref_pose.pose.orientation.w = 1.0;
                break;
            }

            case PID_TRACKING:
            {
                if (!traj_msg_.mpc_ref_points.empty())
                {
                    // Use horizon point 0 as current setpoint with position + velocity feedforward
                    const auto &pt = traj_msg_.mpc_ref_points[0];

                    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                         mavros_msgs::PositionTarget::IGNORE_AFY |
                                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

                    setpoint.position.x = pt.position.x;
                    setpoint.position.y = pt.position.y;
                    setpoint.position.z = pt.position.z;
                    setpoint.velocity.x = pt.velocity.x;
                    setpoint.velocity.y = pt.velocity.y;
                    setpoint.velocity.z = pt.velocity.z;

                    double yaw = std::atan2(pt.velocity.y, pt.velocity.x);
                    setpoint.yaw = yaw;

                    ref_pose.pose.position.x = pt.position.x;
                    ref_pose.pose.position.y = pt.position.y;
                    ref_pose.pose.position.z = pt.position.z;
                    ref_pose.pose.orientation.w = std::cos(yaw * 0.5);
                    ref_pose.pose.orientation.z = std::sin(yaw * 0.5);

                    if (reachGoal(current_odom_, goal_))
                    {
                        mode_ = PID_HOVER;
                        hover_odom_ = current_odom_;
                        ROS_INFO("[PID Baseline] Trajectory complete -> Switched to PID_HOVER");
                    }
                }
                break;
            }
            }

            // Publish PX4 Setpoint
            setpoint_raw_pub_.publish(setpoint);

            // Publish Debug & Telemetry Topics for DataLogger
            std_msgs::Int8 mode_msg;
            mode_msg.data = static_cast<int8_t>(mode_);
            debug_mode_pub_.publish(mode_msg);

            debug_ref_pose_pub_.publish(ref_pose);

            std_msgs::Float32MultiArray ctrl_msg;
            ctrl_msg.data = {9.8066f, 0.0f, 0.0f, 0.0f};
            debug_control_pub_.publish(ctrl_msg);

            std_msgs::Float64 hov_msg;
            hov_msg.data = 0.58;
            debug_hover_thrust_pub_.publish(hov_msg);

            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_baseline_node");
    ros::NodeHandle nh;

    PIDBaselineController controller(nh);
    controller.run();

    return 0;
}
