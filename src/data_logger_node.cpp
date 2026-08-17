#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <sys/stat.h>

class DataLogger
{
private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber ref_pose_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber raw_control_sub_;
    ros::Subscriber hover_thrust_sub_;
    ros::Subscriber cmd_attitude_sub_;
    ros::Subscriber imu_sub_;

    // Publishers for real-time visualization (rqt_plot / PlotJuggler)
    ros::Publisher pub_pos_error_;
    ros::Publisher pub_vel_error_;
    ros::Publisher pub_pos_error_norm_;
    ros::Publisher pub_vel_error_norm_;

    // State Variables
    nav_msgs::Odometry current_odom_;
    geometry_msgs::PoseStamped ref_pose_;
    int mpc_mode_;
    std::vector<float> raw_control_; // [thrust_acc, wx, wy, wz]
    double estimated_hover_thrust_;
    double command_thrust_;
    double imu_acc_z_;
    bool has_odom_, has_ref_;

    // Logging
    std::ofstream csv_file_;
    std::string save_dir_;
    std::string csv_filepath_;
    double log_rate_;
    ros::Time start_time_;
    int record_count_;

public:
    DataLogger(ros::NodeHandle &nh) : nh_(nh), mpc_mode_(-1), estimated_hover_thrust_(0.0),
                                      command_thrust_(0.0), imu_acc_z_(9.8066),
                                      has_odom_(false), has_ref_(false), record_count_(0)
    {
        std::string odom_topic;
        nh_.param<std::string>("/odomTopicName", odom_topic, "/mavros/local_position/odom");
        std::string log_prefix;
        nh_.param<std::string>("log_prefix", log_prefix, "NMPC");
        nh_.param<double>("log_rate", log_rate_, 20.0);
        nh_.param<std::string>("save_dir", save_dir_, "");

        raw_control_.resize(4, 0.0f);

        // Determine CSV file path
        if (save_dir_.empty())
        {
            std::string pkg_path = ros::package::getPath("uav_mpc");
            if (!pkg_path.empty())
            {
                save_dir_ = pkg_path + "/data";
            }
            else
            {
                save_dir_ = "/tmp/uav_mpc_data";
            }
        }

        // Create data directory if not exists
        mkdir(save_dir_.c_str(), 0777);

        // Generate timestamped file name
        std::time_t t = std::time(nullptr);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&t));
        csv_filepath_ = save_dir_ + "/flight_log_" + log_prefix + "_" + std::string(time_str) + ".csv";

        csv_file_.open(csv_filepath_);
        if (csv_file_.is_open())
        {
            // Write CSV Header
            csv_file_ << "timestamp,time_sec,mode,"
                      << "pos_x,pos_y,pos_z,"
                      << "ref_x,ref_y,ref_z,"
                      << "err_x,err_y,err_z,err_pos_norm,"
                      << "vel_x,vel_y,vel_z,"
                      << "ctrl_acc_z,ctrl_wx,ctrl_wy,ctrl_wz,"
                      << "cmd_thrust,estimated_hover_thrust,imu_acc_z\n";
            ROS_INFO("[DataLogger] Recording flight data to: %s", csv_filepath_.c_str());
        }
        else
        {
            ROS_ERROR("[DataLogger] Failed to create log file at: %s", csv_filepath_.c_str());
        }

        // Setup Subscribers
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 10, &DataLogger::odomCallback, this);
        ref_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mpc_debug/ref_pose", 10, &DataLogger::refPoseCallback, this);
        mode_sub_ = nh_.subscribe<std_msgs::Int8>("/mpc_debug/mode", 10, &DataLogger::modeCallback, this);
        raw_control_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/mpc_debug/raw_control", 10, &DataLogger::controlCallback, this);
        hover_thrust_sub_ = nh_.subscribe<std_msgs::Float64>("/mpc_debug/hover_thrust", 10, &DataLogger::hoverThrustCallback, this);
        cmd_attitude_sub_ = nh_.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10, &DataLogger::attitudeCallback, this);
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &DataLogger::imuCallback, this);

        // Setup Real-time Error Publishers
        pub_pos_error_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mpc_debug/pos_error_3d", 1);
        pub_vel_error_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mpc_debug/vel_error_3d", 1);
        pub_pos_error_norm_ = nh_.advertise<std_msgs::Float64>("/mpc_debug/pos_error_norm", 1);
        pub_vel_error_norm_ = nh_.advertise<std_msgs::Float64>("/mpc_debug/vel_error_norm", 1);
    }

    ~DataLogger()
    {
        if (csv_file_.is_open())
        {
            csv_file_.flush();
            csv_file_.close();
            ROS_INFO("[DataLogger] Log saved successfully (%d records) to: %s", record_count_, csv_filepath_.c_str());
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        current_odom_ = *msg;
        has_odom_ = true;
    }

    void refPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ref_pose_ = *msg;
        has_ref_ = true;
    }

    void modeCallback(const std_msgs::Int8::ConstPtr &msg)
    {
        mpc_mode_ = msg->data;
    }

    void controlCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() >= 4)
        {
            raw_control_ = msg->data;
        }
    }

    void hoverThrustCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        estimated_hover_thrust_ = msg->data;
    }

    void attitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
    {
        command_thrust_ = msg->thrust;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        imu_acc_z_ = msg->linear_acceleration.z;
    }

    void spin()
    {
        ros::Rate rate(log_rate_);
        start_time_ = ros::Time::now();
        ros::Time last_print_time = ros::Time::now();

        while (ros::ok())
        {
            ros::spinOnce();
            ros::Time now = ros::Time::now();
            double time_sec = (now - start_time_).toSec();

            if (has_odom_ && has_ref_)
            {
                // Position error (ref - real)
                double ex = ref_pose_.pose.position.x - current_odom_.pose.pose.position.x;
                double ey = ref_pose_.pose.position.y - current_odom_.pose.pose.position.y;
                double ez = ref_pose_.pose.position.z - current_odom_.pose.pose.position.z;
                double pos_err_norm = std::sqrt(ex * ex + ey * ey + ez * ez);

                // Publish real-time error messages
                geometry_msgs::Vector3Stamped pos_err_msg;
                pos_err_msg.header.stamp = now;
                pos_err_msg.header.frame_id = "world";
                pos_err_msg.vector.x = ex;
                pos_err_msg.vector.y = ey;
                pos_err_msg.vector.z = ez;
                pub_pos_error_.publish(pos_err_msg);

                std_msgs::Float64 norm_msg;
                norm_msg.data = pos_err_norm;
                pub_pos_error_norm_.publish(norm_msg);

                // Write to CSV file
                if (csv_file_.is_open())
                {
                    csv_file_ << std::fixed << std::setprecision(4)
                              << now.toSec() << ","
                              << time_sec << ","
                              << mpc_mode_ << ","
                              << current_odom_.pose.pose.position.x << ","
                              << current_odom_.pose.pose.position.y << ","
                              << current_odom_.pose.pose.position.z << ","
                              << ref_pose_.pose.position.x << ","
                              << ref_pose_.pose.position.y << ","
                              << ref_pose_.pose.position.z << ","
                              << ex << "," << ey << "," << ez << ","
                              << pos_err_norm << ","
                              << current_odom_.twist.twist.linear.x << ","
                              << current_odom_.twist.twist.linear.y << ","
                              << current_odom_.twist.twist.linear.z << ","
                              << raw_control_[0] << ","
                              << raw_control_[1] << ","
                              << raw_control_[2] << ","
                              << raw_control_[3] << ","
                              << command_thrust_ << ","
                              << estimated_hover_thrust_ << ","
                              << imu_acc_z_ << "\n";
                    record_count_++;
                }

                // Console display at 1Hz
                if ((now - last_print_time).toSec() >= 1.0)
                {
                    last_print_time = now;
                    std::string mode_str = (mpc_mode_ == 0) ? "TAKEOFF" : (mpc_mode_ == 1) ? "HOVER" : (mpc_mode_ == 2) ? "TRACKING" : "WAIT";
                    ROS_INFO("[Logger] Mode: %-8s | PosErr: %.3f m (X:%.2f, Y:%.2f, Z:%.2f) | Thr: %.3f | HovEst: %.3f",
                             mode_str.c_str(), pos_err_norm, ex, ey, ez, command_thrust_, estimated_hover_thrust_);
                }
            }

            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_logger_node");
    ros::NodeHandle nh("~");

    DataLogger logger(nh);
    logger.spin();

    return 0;
}
