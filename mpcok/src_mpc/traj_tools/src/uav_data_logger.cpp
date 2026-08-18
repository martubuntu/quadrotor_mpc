#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <quadrotor_msgs/mpc_ref_traj.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

class UAVDataLogger
{
public:
    UAVDataLogger()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "output_file",
            output_file_,
            "/tmp/uav_log.csv");

        pnh_.param("rate", log_rate_, 20.0);

        nan_ = std::numeric_limits<double>::quiet_NaN();

        mode_ = -1;
        hover_thrust_ = nan_;
        thrust_cmd_ = nan_;
        cmd_wx_ = cmd_wy_ = cmd_wz_ = nan_;

        for(int i = 0; i < 10; ++i)
            x0_[i] = nan_;

        for(int i = 0; i < 4; ++i)
            control_[i] = nan_;

        ref_px_ = ref_py_ = ref_pz_ = nan_;
        ref_vx_ = ref_vy_ = ref_vz_ = nan_;
        ref_ax_ = ref_ay_ = ref_az_ = nan_;

        ref_qw_ = ref_qx_ = ref_qy_ = ref_qz_ = nan_;

        eso_dx_ = nan_;
        eso_dy_ = nan_;
        eso_dz_ = nan_;

        x0_sub_ = nh_.subscribe(
            "/mpc_debug/acado_x0",
            10,
            &UAVDataLogger::x0Callback,
            this);

        control_sub_ = nh_.subscribe(
            "/mpc_debug/raw_control",
            10,
            &UAVDataLogger::controlCallback,
            this);

        hover_sub_ = nh_.subscribe(
            "/mpc_debug/hover_thrust",
            10,
            &UAVDataLogger::hoverCallback,
            this);

        cmd_sub_ = nh_.subscribe(
            "/mavros/setpoint_raw/attitude",
            10,
            &UAVDataLogger::cmdCallback,
            this);

        mode_sub_ = nh_.subscribe(
            "/mpc_debug/mode",
            10,
            &UAVDataLogger::modeCallback,
            this);

        ref_pose_sub_ = nh_.subscribe(
            "/mpc_debug/ref_pose",
            10,
            &UAVDataLogger::refPoseCallback,
            this);

        traj_sub_ = nh_.subscribe(
            "/mpc_ref_traj",
            10,
            &UAVDataLogger::trajCallback,
            this);

        eso_sub_ = nh_.subscribe(
            "/eso/disturbance",
            10,
            &UAVDataLogger::esoCallback,
            this);

        file_.open(output_file_.c_str());

        if(!file_.is_open())
        {
            ROS_FATAL("Cannot open output file: %s",
                      output_file_.c_str());
            ros::shutdown();
            return;
        }

        file_ << std::setprecision(10);

        writeHeader();

        start_time_ = ros::Time::now();

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / log_rate_),
            &UAVDataLogger::timerCallback,
            this);

        ROS_INFO("UAV logger started.");
        ROS_INFO("Output: %s", output_file_.c_str());
        ROS_INFO("Rate: %.1f Hz", log_rate_);
    }

    ~UAVDataLogger()
    {
        if(file_.is_open())
        {
            file_.flush();
            file_.close();
        }
    }

private:

    void writeHeader()
    {
        file_
        << "time,"
        << "mode,"

        << "px,py,pz,"
        << "qw,qx,qy,qz,"
        << "vx,vy,vz,"

        << "ref_px,ref_py,ref_pz,"
        << "ref_vx,ref_vy,ref_vz,"
        << "ref_ax,ref_ay,ref_az,"
        << "ref_qw,ref_qx,ref_qy,ref_qz,"

        << "aT,wx,wy,wz,"

        << "thrust_cmd,"
        << "cmd_wx,cmd_wy,cmd_wz,"

        << "hover_thrust,"

        << "eso_dx,eso_dy,eso_dz"

        << "\n";
    }

    void x0Callback(
        const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() < 10)
            return;

        for(int i = 0; i < 10; ++i)
            x0_[i] = msg->data[i];
    }

    void controlCallback(
        const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() < 4)
            return;

        for(int i = 0; i < 4; ++i)
            control_[i] = msg->data[i];
    }

    void hoverCallback(
        const std_msgs::Float64::ConstPtr& msg)
    {
        hover_thrust_ = msg->data;
    }

    void modeCallback(
        const std_msgs::Int8::ConstPtr& msg)
    {
        mode_ = msg->data;
    }

    void cmdCallback(
        const mavros_msgs::AttitudeTarget::ConstPtr& msg)
    {
        thrust_cmd_ = msg->thrust;

        cmd_wx_ = msg->body_rate.x;
        cmd_wy_ = msg->body_rate.y;
        cmd_wz_ = msg->body_rate.z;
    }

    void refPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ref_qw_ = msg->pose.orientation.w;
        ref_qx_ = msg->pose.orientation.x;
        ref_qy_ = msg->pose.orientation.y;
        ref_qz_ = msg->pose.orientation.z;
    }

    void trajCallback(
        const quadrotor_msgs::mpc_ref_traj::ConstPtr& msg)
    {
        if(msg->mpc_ref_points.empty())
            return;

        const auto& p = msg->mpc_ref_points.front();

        ref_px_ = p.position.x;
        ref_py_ = p.position.y;
        ref_pz_ = p.position.z;

        ref_vx_ = p.velocity.x;
        ref_vy_ = p.velocity.y;
        ref_vz_ = p.velocity.z;

        ref_ax_ = p.acceleration.x;
        ref_ay_ = p.acceleration.y;
        ref_az_ = p.acceleration.z;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const double t =
            (ros::Time::now() - start_time_).toSec();

        file_
        << t << ","
        << static_cast<int>(mode_) << ","

        // x0
        << x0_[0] << ","
        << x0_[1] << ","
        << x0_[2] << ","

        << x0_[3] << ","
        << x0_[4] << ","
        << x0_[5] << ","
        << x0_[6] << ","

        << x0_[7] << ","
        << x0_[8] << ","
        << x0_[9] << ","

        // reference
        << ref_px_ << ","
        << ref_py_ << ","
        << ref_pz_ << ","

        << ref_vx_ << ","
        << ref_vy_ << ","
        << ref_vz_ << ","

        << ref_ax_ << ","
        << ref_ay_ << ","
        << ref_az_ << ","

        << ref_qw_ << ","
        << ref_qx_ << ","
        << ref_qy_ << ","
        << ref_qz_ << ","

        // NMPC
        << control_[0] << ","
        << control_[1] << ","
        << control_[2] << ","
        << control_[3] << ","

        // PX4 command
        << thrust_cmd_ << ","
        << cmd_wx_ << ","
        << cmd_wy_ << ","
        << cmd_wz_ << ","

        << hover_thrust_ << ","
        << eso_dx_ << ","
        << eso_dy_ << ","
        << eso_dz_
        << "\n";

        count_++;

        // 每秒左右写磁盘一次，避免每行 flush
        if(count_ % 20 == 0)
            file_.flush();
    }

    void esoCallback(
        const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        eso_dx_ = msg->vector.x;
        eso_dy_ = msg->vector.y;
        eso_dz_ = msg->vector.z;

        ROS_INFO_THROTTLE(
            1.0,
            "LOGGER CALLBACK ESO = [%.3f, %.3f, %.3f]",
            eso_dx_,
            eso_dy_,
            eso_dz_);
    }
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber x0_sub_;
    ros::Subscriber control_sub_;
    ros::Subscriber hover_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber ref_pose_sub_;
    ros::Subscriber traj_sub_;

    ros::Timer timer_;

    std::ofstream file_;

    std::string output_file_;
    double log_rate_;

    ros::Time start_time_;

    int8_t mode_;

    double x0_[10];
    double control_[4];

    double hover_thrust_;

    double thrust_cmd_;
    double cmd_wx_;
    double cmd_wy_;
    double cmd_wz_;

    double ref_px_, ref_py_, ref_pz_;
    double ref_vx_, ref_vy_, ref_vz_;
    double ref_ax_, ref_ay_, ref_az_;

    double ref_qw_, ref_qx_, ref_qy_, ref_qz_;

    double nan_;

    ros::Subscriber eso_sub_;

    double eso_dx_;
    double eso_dy_;
    double eso_dz_;

    unsigned long count_ = 0;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_data_logger");

    UAVDataLogger logger;

    ros::spin();

    return 0;
}
