#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <cmath>
#include <filesystem>

class DataLoggerNode : public rclcpp::Node
{
public:
  DataLoggerNode()
  : Node("data_logger_node"),
    record_count_(0),
    accumulated_energy_j_(0.0),
    has_odom_(false),
    has_ref_(false),
    has_battery_(false),
    has_eso_(false),
    mpc_mode_(-1),
    command_thrust_(0.0),
    estimated_hover_thrust_(0.72),
    solve_time_ms_(0.0),
    battery_voltage_(0.0),
    battery_current_(0.0),
    battery_power_(0.0),
    imu_acc_z_(9.8066),
    imu_wx_(0.0), imu_wy_(0.0), imu_wz_(0.0),
    eso_dx_(0.0), eso_dy_(0.0), eso_dz_(0.0)
  {
    raw_control_.resize(4, 0.0f);

    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/mavros/local_position/odom");
    const auto ref_topic = declare_parameter<std::string>(
      "ref_topic", "/circle_traj/current_ref");
    const auto battery_topic = declare_parameter<std::string>(
      "battery_topic", "/mavros/battery");
    const auto log_prefix = declare_parameter<std::string>(
      "log_prefix", "NMPC");
    const double log_rate = declare_parameter<double>(
      "log_rate", 30.0);
    save_dir_ = declare_parameter<std::string>(
      "save_dir", "data");

    // Ensure output directory exists
    try {
      std::filesystem::create_directories(save_dir_);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not create directory %s: %s", save_dir_.c_str(), e.what());
      save_dir_ = ".";
    }

    // Generate timestamped CSV filepath
    const auto now_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm_buf;
#if defined(_WIN32)
    localtime_s(&tm_buf, &now_time);
#else
    localtime_r(&now_time, &tm_buf);
#endif
    char time_str[64];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &tm_buf);
    csv_filepath_ = save_dir_ + "/flight_log_" + log_prefix + "_" + std::string(time_str) + ".csv";

    csv_file_.open(csv_filepath_);
    if (csv_file_.is_open()) {
      csv_file_ << std::fixed << std::setprecision(6);
      csv_file_ << "timestamp,time_sec,mode,"
                << "pos_x,pos_y,pos_z,"
                << "ref_x,ref_y,ref_z,"
                << "err_x,err_y,err_z,err_pos_norm,"
                << "vel_x,vel_y,vel_z,"
                << "roll_deg,pitch_deg,yaw_deg,"
                << "ref_roll_deg,ref_pitch_deg,ref_yaw_deg,"
                << "err_roll_deg,err_pitch_deg,err_yaw_deg,"
                << "ctrl_acc_z,ctrl_wx,ctrl_wy,ctrl_wz,"
                << "cmd_thrust,estimated_hover_thrust,solve_time_ms,"
                << "voltage_v,current_a,power_w,energy_j,"
                << "imu_acc_z,imu_wx,imu_wy,imu_wz,"
                << "eso_dx,eso_dy,eso_dz\n";
      RCLCPP_INFO(get_logger(), "[DataLogger] Recording CSV flight data to: %s", csv_filepath_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "[DataLogger] Failed to create log file: %s", csv_filepath_.c_str());
    }

    // Setup Subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        has_odom_ = true;
      });

    ref_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ref_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        ref_pose_ = *msg;
        has_ref_ = true;
      });

    mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/mpc_debug/mode", rclcpp::QoS(10),
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        mpc_mode_ = msg->data;
      });

    raw_control_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/mpc_debug/raw_control", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
          raw_control_ = msg->data;
        }
      });

    hover_thrust_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mpc_debug/hover_thrust", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        estimated_hover_thrust_ = msg->data;
      });

    solve_time_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mpc_debug/solve_time_ms", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        solve_time_ms_ = msg->data;
      });

    cmd_attitude_sub_ = create_subscription<mavros_msgs::msg::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", rclcpp::QoS(10),
      [this](const mavros_msgs::msg::AttitudeTarget::SharedPtr msg) {
        command_thrust_ = msg->thrust;
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_acc_z_ = msg->linear_acceleration.z;
        imu_wx_ = msg->angular_velocity.x;
        imu_wy_ = msg->angular_velocity.y;
        imu_wz_ = msg->angular_velocity.z;
      });

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        battery_voltage_ = msg->voltage;
        battery_current_ = std::abs(msg->current); // discharge current magnitude
        battery_power_ = battery_voltage_ * battery_current_;
        has_battery_ = true;
      });

    eso_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/eso/disturbance", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        eso_dx_ = msg->vector.x;
        eso_dy_ = msg->vector.y;
        eso_dz_ = msg->vector.z;
        has_eso_ = true;
      });

    start_time_ = now();
    last_log_time_ = now();

    const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, log_rate)));
    timer_ = create_wall_timer(period, std::bind(&DataLoggerNode::logCycle, this));
  }

  ~DataLoggerNode() override
  {
    if (csv_file_.is_open()) {
      csv_file_.flush();
      csv_file_.close();
      RCLCPP_INFO(
        get_logger(),
        "[DataLogger] Log saved successfully (%d records, Energy=%.2f J) to: %s",
        record_count_, accumulated_energy_j_, csv_filepath_.c_str());
    }
  }

private:
  void quatToEulerDeg(const Eigen::Quaterniond & q, double & roll_deg, double & pitch_deg, double & yaw_deg) const
  {
    const double roll_rad = std::atan2(
      2.0 * (q.w() * q.x() + q.y() * q.z()),
      1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    const double pitch_rad = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    const double yaw_rad = std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    roll_deg = roll_rad * 180.0 / M_PI;
    pitch_deg = pitch_rad * 180.0 / M_PI;
    yaw_deg = yaw_rad * 180.0 / M_PI;
  }

  void logCycle()
  {
    if (!has_odom_ || !csv_file_.is_open()) {
      return;
    }

    const auto current_time = now();
    const double time_sec = (current_time - start_time_).seconds();
    const double dt = (current_time - last_log_time_).seconds();
    last_log_time_ = current_time;

    if (has_battery_ && dt > 0.0 && dt < 1.0) {
      accumulated_energy_j_ += battery_power_ * dt;
    }

    // Actual position & velocity
    const double px = current_odom_.pose.pose.position.x;
    const double py = current_odom_.pose.pose.position.y;
    const double pz = current_odom_.pose.pose.position.z;
    const double vx = current_odom_.twist.twist.linear.x;
    const double vy = current_odom_.twist.twist.linear.y;
    const double vz = current_odom_.twist.twist.linear.z;

    // Actual attitude
    const Eigen::Quaterniond q_act(
      current_odom_.pose.pose.orientation.w,
      current_odom_.pose.pose.orientation.x,
      current_odom_.pose.pose.orientation.y,
      current_odom_.pose.pose.orientation.z);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quatToEulerDeg(q_act, roll, pitch, yaw);

    // Reference position & attitude
    double rx = px, ry = py, rz = pz;
    double ref_roll = 0.0, ref_pitch = 0.0, ref_yaw = yaw;
    if (has_ref_) {
      rx = ref_pose_.pose.position.x;
      ry = ref_pose_.pose.position.y;
      rz = ref_pose_.pose.position.z;
      const Eigen::Quaterniond q_ref(
        ref_pose_.pose.orientation.w,
        ref_pose_.pose.orientation.x,
        ref_pose_.pose.orientation.y,
        ref_pose_.pose.orientation.z);
      if (std::abs(q_ref.norm() - 1.0) < 0.1) {
        quatToEulerDeg(q_ref, ref_roll, ref_pitch, ref_yaw);
      }
    }

    // Tracking errors
    const double ex = px - rx;
    const double ey = py - ry;
    const double ez = pz - rz;
    const double err_pos_norm = std::sqrt(ex * ex + ey * ey + ez * ez);
    const double err_roll = roll - ref_roll;
    const double err_pitch = pitch - ref_pitch;
    double err_yaw = yaw - ref_yaw;
    while (err_yaw > 180.0) err_yaw -= 360.0;
    while (err_yaw < -180.0) err_yaw += 360.0;

    csv_file_ << current_time.nanoseconds() << "," << time_sec << "," << mpc_mode_ << ","
              << px << "," << py << "," << pz << ","
              << rx << "," << ry << "," << rz << ","
              << ex << "," << ey << "," << ez << "," << err_pos_norm << ","
              << vx << "," << vy << "," << vz << ","
              << roll << "," << pitch << "," << yaw << ","
              << ref_roll << "," << ref_pitch << "," << ref_yaw << ","
              << err_roll << "," << err_pitch << "," << err_yaw << ","
              << raw_control_[0] << "," << raw_control_[1] << "," << raw_control_[2] << "," << raw_control_[3] << ","
              << command_thrust_ << "," << estimated_hover_thrust_ << "," << solve_time_ms_ << ","
              << battery_voltage_ << "," << battery_current_ << "," << battery_power_ << "," << accumulated_energy_j_ << ","
              << imu_acc_z_ << "," << imu_wx_ << "," << imu_wy_ << "," << imu_wz_ << ","
              << eso_dx_ << "," << eso_dy_ << "," << eso_dz_ << "\n";

    record_count_++;
    if (record_count_ % 150 == 0) {
      csv_file_.flush();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[DataLogger] Logged %d samples | PosErr: %.3fm | Power: %.1fW | Energy: %.1fJ",
        record_count_, err_pos_norm, battery_power_, accumulated_energy_j_);
    }
  }

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr raw_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hover_thrust_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr solve_time_sub_;
  rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr cmd_attitude_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr eso_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::Odometry current_odom_;
  geometry_msgs::msg::PoseStamped ref_pose_;
  std::vector<float> raw_control_;
  std::ofstream csv_file_;
  std::string save_dir_;
  std::string csv_filepath_;
  rclcpp::Time start_time_;
  rclcpp::Time last_log_time_;

  int record_count_;
  double accumulated_energy_j_;
  bool has_odom_;
  bool has_ref_;
  bool has_battery_;
  bool has_eso_;
  int mpc_mode_;
  double command_thrust_;
  double estimated_hover_thrust_;
  double solve_time_ms_;
  double battery_voltage_;
  double battery_current_;
  double battery_power_;
  double imu_acc_z_;
  double imu_wx_, imu_wy_, imu_wz_;
  double eso_dx_, eso_dy_, eso_dz_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
