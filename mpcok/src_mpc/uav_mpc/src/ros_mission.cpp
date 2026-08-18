#include "uav_mpc/ros_mission.h"

Eigen::Vector3f goal;
Eigen::Quaternionf Q;
Eigen::Vector3f eulerAngle;

Eigen::MatrixXd reference(Nreference, Ksample + 1);

MPCRos::MPCRos(ros::NodeHandle &nh):nh(nh)
{
  nh.getParam("/mass", mass);
  nh.getParam("/hover_thrust", hover_thrust);
  nh.getParam("/takeoff_height", takeoff_height);
  nh.getParam("/ctrl_hz", ctrl_hz);
  nh.getParam("/odomTopicName", odomTopicName);

  mpc_mode = AUTO_TAKEOFF;
  reference = Eigen::MatrixXd::Zero(Nreference, Ksample + 1);
  thrust_estimator = new ThrustEstimator(hover_thrust, 9.8066);
}

MPCRos::~MPCRos()
{
}

void MPCRos::ExectControl()
{
  arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
  odom_sub = nh.subscribe<nav_msgs::Odometry> (odomTopicName, 1, &MPCRos::odom_Callback, this);
  state_sub = nh.subscribe<mavros_msgs::State> ("/mavros/state", 10, &MPCRos::state_Callback, this);
  traj_sub = nh.subscribe<quadrotor_msgs::mpc_ref_traj> ("/mpc_ref_traj", 1, &MPCRos::traj_Callback, this);
  imu_sub = nh.subscribe<sensor_msgs::Imu> ("/mavros/imu/data", 10, &MPCRos::imu_Callback, this);
  eso_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/eso/disturbance",10,&MPCRos::eso_Callback,this);
  cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude", 1);
  debug_mode_pub = nh.advertise<std_msgs::Int8>("/mpc_debug/mode", 1);
  debug_ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mpc_debug/ref_pose", 1);
  debug_control_pub = nh.advertise<std_msgs::Float32MultiArray>("/mpc_debug/raw_control", 1);
  debug_hover_thrust_pub = nh.advertise<std_msgs::Float64>("/mpc_debug/hover_thrust", 1);

  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;

  ros::Rate rate(ctrl_hz);

  wrapper = new MPCWrapper(nh);
  last_request = ros::Time::now();

  while(ros::ok())
  {
    FSMProcess();
    ros::spinOnce();
    rate.sleep();
  }
}

void MPCRos::FSMProcess()
{
  if(odom_flag)
  {
    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0)))
    {
      if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        ROS_INFO("Offboard enabled");
      last_request = ros::Time::now();
    } 
    else 
    {
      if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
      {
        if(arming_client.call(arm_cmd) && arm_cmd.response.success)
          ROS_INFO("Vehicle armed");
        last_request = ros::Time::now();
      }
    }

    if(!mpc_init)
    {
      mpc_init = wrapper->initSolver(start_odom);
      ROS_INFO("MPC_INIT");
    }

    if(mpc_init)
    {
      reference = Eigen::MatrixXd::Zero(Nreference, Ksample + 1);
      //ROS_INFO("mpc_mode: %d", mpc_mode);
      switch(mpc_mode)
      {
        case AUTO_HOVER:
        {
            if(fsm_switch)
            {
              ROS_INFO("AUTO_HOVER");
              fsm_switch = 0;
            }

            Q = Eigen::Quaternionf(
                hover_odom.pose.pose.orientation.w,
                hover_odom.pose.pose.orientation.x,
                hover_odom.pose.pose.orientation.y,
                hover_odom.pose.pose.orientation.z);

            eulerAngle = Q.matrix().eulerAngles(2, 1, 0);

            if(eulerAngle(0) > 1.5707963)
              eulerAngle(0) = eulerAngle(0) - 3.1415926;

            Eigen::Quaternionf q_ref(
                cos(eulerAngle(0) / 2.0),
                0.0,
                0.0,
                sin(eulerAngle(0) / 2.0));

            // 保证参考四元数和当前四元数处于同一半球
            if(q_ref.coeffs().dot(Q.coeffs()) < 0.0)
            {
              q_ref.coeffs() *= -1.0;
            }

            for(int i = 0; i < Ksample + 1; ++i)
            {
              reference.col(i) <<
                  hover_odom.pose.pose.position.x,
                  hover_odom.pose.pose.position.y,
                  hover_odom.pose.pose.position.z,

                  q_ref.w(),
                  q_ref.x(),
                  q_ref.y(),
                  q_ref.z(),

                  0, 0, 0,
                  9.8066, 0, 0, 0;
            }

            wrapper->gerReference(reference);

            if(wrapper->getSolution(current_odom, control))
              publishcontrol();
            else
              exit(0);

            break;
        }

        case AUTO_TRACKING:
       {
          if(fsm_switch) 
          {
            ROS_INFO("AUTO_TRACKING");
            fsm_switch = 0;
          }
          getTrajRef();
          if(reachgoal(current_odom, goal))
          {
            mpc_mode = AUTO_HOVER;
            fsm_switch = 1;
          }
          wrapper->gerReference(reference);
          if(wrapper->getSolution(current_odom, control))
            publishcontrol();
          else
          {
            mpc_mode = AUTO_HOVER;
            fsm_switch = 1;
            ROS_ERROR("NO_Solution. Turn to HOVER Mode");
          }
          break;
        }

        case AUTO_TAKEOFF:
      {
          if(fsm_switch) 
          {
            ROS_INFO("AUTO_TAKEOFF");
            fsm_switch = 0;
          }
          for(int i = 0; i < Ksample + 1; ++i)
          {
            reference.col(i) << start_odom.pose.pose.position.x, start_odom.pose.pose.position.y, takeoff_height,
                                start_odom.pose.pose.orientation.w, start_odom.pose.pose.orientation.x, start_odom.pose.pose.orientation.y, start_odom.pose.pose.orientation.z,
                                0, 0, 0,
                                9.8066, 0, 0, 0;
          }
//          Eigen::Vector3f hover_point(start_odom.pose.pose.position.x, start_odom.pose.pose.position.y, takeoff_height);
//          if(reachgoal(current_odom, hover_point))
//          {
//            mpc_mode = AUTO_HOVER;
//            fsm_switch = 1;
//          }
          double z_error =
              std::fabs(current_odom.pose.pose.position.z - takeoff_height);

          if(z_error < 0.08)
          {
              // 记录风扰下实际悬停位置和姿态
              hover_odom = current_odom;

              mpc_mode = AUTO_HOVER;
              fsm_switch = 1;
          }
          wrapper->gerReference(reference);
          if(wrapper->getSolution(current_odom, control))
            publishcontrol();
          // if(control[0] > 0.45 * 9.8066 / hover_thrust)
          //   control[0] = 0.45 * 9.8066 / hover_thrust;
          break;
      }
      }
    }
  }
  else
  {
    ROS_WARN("No Odom");
  }
 
}

void MPCRos::odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(!odom_flag)
  {
    start_odom = *msg;
    ROS_INFO("Odom Recieved");
  }

  odom_flag = 1;
  current_odom = *msg;
  // std::cout << "Current odom position:\n" 
  //                         << current_odom.pose.pose.position.x << std::endl 
  //                         << current_odom.pose.pose.position.y << std::endl 
  //                         << current_odom.pose.pose.position.z << std::endl;
}

void MPCRos::state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

void MPCRos::imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // 增加安全逻辑：仅在飞机起桨且处于 OFFBOARD 模式时，才更新推力模型
  if (!current_state.armed || current_state.mode != "OFFBOARD")
  {
    return;
  }
  //if (thrust_estimator)
  bool updated = thrust_estimator->estimateThrustModel(msg->header.stamp.toSec(), msg->linear_acceleration.z);
  if (updated)
  {
    //hover_thrust = 9.8066 / thrust_estimator->getThr2Acc();
  }
}

void MPCRos::eso_Callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  eso_disturbance.x() = msg->vector.x;
  eso_disturbance.y() = msg->vector.y;
  eso_disturbance.z() = msg->vector.z;

  eso_stamp = ros::Time::now();
  eso_received = true;
}

void MPCRos::traj_Callback(const quadrotor_msgs::mpc_ref_traj::ConstPtr& msg)
{
  traj_msg = *msg;

  ROS_INFO("Trajectory");

  goal << msg->goal.x, msg->goal.y, msg->goal.z;
  
  if(mpc_mode == AUTO_HOVER && !(reachgoal(current_odom, goal)))
  {
    mpc_mode = AUTO_TRACKING;
    fsm_switch = 1;
    ROS_INFO("Trajectory Recieved");
  }
}

void MPCRos::getTrajRef()
{
  // ============================================================
  // 1. 固定 yaw：使用进入 HOVER 时的航向
  // ============================================================
  Eigen::Quaterniond q_hover(
      hover_odom.pose.pose.orientation.w,
      hover_odom.pose.pose.orientation.x,
      hover_odom.pose.pose.orientation.y,
      hover_odom.pose.pose.orientation.z);

  q_hover.normalize();

  double yaw_hold = std::atan2(
      2.0 * (q_hover.w() * q_hover.z()
           + q_hover.x() * q_hover.y()),
      1.0 - 2.0 * (q_hover.y() * q_hover.y()
                 + q_hover.z() * q_hover.z()));

  // ============================================================
  // 2. 保存21个参考四元数和 aT_ref
  // ============================================================
  std::vector<Eigen::Vector4d> q_ref_list(Ksample + 1);
  std::vector<double> aT_ref_list(Ksample + 1, 9.8066);

  Eigen::Vector3d acc;
  Eigen::Vector4d quat;

  // 用当前姿态作为四元数符号连续性的起点
  Eigen::Vector4d last_quat;

  last_quat <<
      current_odom.pose.pose.orientation.w,
      current_odom.pose.pose.orientation.x,
      current_odom.pose.pose.orientation.y,
      current_odom.pose.pose.orientation.z;

  if(last_quat.norm() > 1e-6)
  {
    last_quat.normalize();
  }

  int num_points =
      std::min(
          static_cast<int>(traj_msg.mpc_ref_points.size()),
          Ksample + 1);
  Eigen::Vector3d d_hat = Eigen::Vector3d::Zero();

  if(eso_received)
  {
      double age =
          (ros::Time::now() - eso_stamp).toSec();

      if(age >= 0.0 && age < 0.2)
      {
          d_hat = eso_disturbance;
      }
  }

  ROS_INFO_THROTTLE(
      1.0,
      "TRAJ ESO d_hat = [%.3f %.3f %.3f]",
      d_hat.x(),
      d_hat.y(),
      d_hat.z());

  // ============================================================
  // 第一遍：计算所有 q_ref 和 aT_ref
  // ============================================================
  for(int k = 0; k < num_points; ++k)
  {
    const auto& point = traj_msg.mpc_ref_points[k];

    // 注意：
    // 这里暂时不使用 -d_hat 前馈
    acc <<
        point.acceleration.x,
        point.acceleration.y,
        point.acceleration.z + 9.8066;

    // 根据总期望加速度 + 固定yaw生成姿态参考
    acc2quaternion(acc, yaw_hold, quat);

    if(quat.norm() > 1e-6)
    {
      quat.normalize();
    }

    // q 和 -q 表示同一姿态
    // 保证整个预测区间四元数连续
    if(quat.dot(last_quat) < 0.0)
    {
      quat = -quat;
    }

    q_ref_list[k] = quat;
    aT_ref_list[k] = acc.norm();

    last_quat = quat;
  }

  // ============================================================
  // 第二遍：利用相邻 q_ref 计算 body-rate reference
  // ============================================================
  for(int k = 0; k < num_points; ++k)
  {
    const auto& point = traj_msg.mpc_ref_points[k];

    Eigen::Vector3d omega_ref =
        Eigen::Vector3d::Zero();

    // terminal节点没有下一节点，因此不需要计算body rate
    if(k < num_points - 1)
    {
      Eigen::Quaterniond q_k(
          q_ref_list[k][0],
          q_ref_list[k][1],
          q_ref_list[k][2],
          q_ref_list[k][3]);

      Eigen::Quaterniond q_next(
          q_ref_list[k + 1][0],
          q_ref_list[k + 1][1],
          q_ref_list[k + 1][2],
          q_ref_list[k + 1][3]);

      q_k.normalize();
      q_next.normalize();

      /*
       * 对你当前四元数动力学：
       *
       * q_dot = 0.5 * q ⊗ [0, omega_body]
       *
       * 因此：
       *
       * dq = q_k^{-1} q_{k+1}
       *
       * dq 对应的旋转向量 / dt
       * 就近似为 body-rate reference。
       */
      Eigen::Quaterniond dq =
          q_k.conjugate() * q_next;

      dq.normalize();

      // 使用最短旋转
      if(dq.w() < 0.0)
      {
        dq.coeffs() *= -1.0;
      }

      Eigen::AngleAxisd aa(dq);

      omega_ref =
          aa.axis() * aa.angle() / 0.1;
    }
    if(k == 0)
    {
      ROS_INFO_THROTTLE(
          1.0,
          "omega_ref = [%.4f, %.4f, %.4f], aT_ref = %.4f",
          omega_ref.x(),
          omega_ref.y(),
          omega_ref.z(),
          aT_ref_list[k]);
    }
    // ==========================================================
    // 写入 NMPC reference
    // ==========================================================
    reference.col(k) <<
        point.position.x,
        point.position.y,
        point.position.z,

        q_ref_list[k][0],
        q_ref_list[k][1],
        q_ref_list[k][2],
        q_ref_list[k][3],

        point.velocity.x,
        point.velocity.y,
        point.velocity.z,

        aT_ref_list[k],

        omega_ref.x(),
        omega_ref.y(),
        omega_ref.z();
  }
}

void MPCRos::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw, Eigen::Vector4d &quat) 
{
  Eigen::Vector3d zb_des, yb_des, xb_des, yc;
  Eigen::Matrix3d R;

  yc = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*Eigen::Vector3d::UnitY();
  zb_des = vector_acc / vector_acc.norm();
  xb_des = yc.cross(zb_des) / ( yc.cross(zb_des) ).norm();
  yb_des = zb_des.cross(xb_des) / (zb_des.cross(xb_des)).norm();

  R << xb_des(0), yb_des(0), zb_des(0), 
       xb_des(1), yb_des(1), zb_des(1), 
       xb_des(2), yb_des(2), zb_des(2);

  double tr = R.trace();
  if (tr > 0.0) 
  {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat[0] = 0.25 * S;
    quat[1] = (R(2, 1) - R(1, 2)) / S;
    quat[2] = (R(0, 2) - R(2, 0)) / S;
    quat[3] = (R(1, 0) - R(0, 1)) / S;
  } 
  else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) 
  {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat[0] = (R(2, 1) - R(1, 2)) / S;
    quat[1] = 0.25 * S;
    quat[2] = (R(0, 1) + R(1, 0)) / S;
    quat[3] = (R(0, 2) + R(2, 0)) / S;
  } 
  else if (R(1, 1) > R(2, 2)) 
  {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat[0] = (R(0, 2) - R(2, 0)) / S;
    quat[1] = (R(0, 1) + R(1, 0)) / S;
    quat[2] = 0.25 * S;
    quat[3] = (R(1, 2) + R(2, 1)) / S;
  } 
  else 
  {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat[0] = (R(1, 0) - R(0, 1)) / S;
    quat[1] = (R(0, 2) + R(2, 0)) / S;
    quat[2] = (R(1, 2) + R(2, 1)) / S;
    quat[3] = 0.25 * S;
  }
}

bool MPCRos::reachgoal(nav_msgs::Odometry& msg, Eigen::Vector3f& goal)
{
  double distance;
  distance = (msg.pose.pose.position.x - goal[0]) * (msg.pose.pose.position.x - goal[0]) + 
             (msg.pose.pose.position.y - goal[1]) * (msg.pose.pose.position.y - goal[1]) + 
             (msg.pose.pose.position.z - goal[2]) * (msg.pose.pose.position.z - goal[2]);
  // distance = (msg.pose.pose.position.x - goal[0]) * (msg.pose.pose.position.x - goal[0]) + 
  //            (msg.pose.pose.position.y - goal[1]) * (msg.pose.pose.position.y - goal[1]) + 
  //            (msg.pose.pose.position.z - goal[2] + 1) * (msg.pose.pose.position.z - goal[2] + 1);

  if(distance < 0.08*0.08)
  {
    hover_odom = current_odom;
    return true;
  }
  else
    return false;
}

void MPCRos::publishcontrol()
{
  // 1. Publish MPC Mode (0: Takeoff, 1: Hover, 2: Tracking)
  std_msgs::Int8 mode_msg;
  mode_msg.data = mpc_mode;
  debug_mode_pub.publish(mode_msg);

  // 2. Publish Current Reference Pose (Horizon Step 0)
  geometry_msgs::PoseStamped ref_pose;
  ref_pose.header.stamp = ros::Time::now();
  ref_pose.header.frame_id = "world"; 
  ref_pose.pose.position.x = reference(0, 0);
  ref_pose.pose.position.y = reference(1, 0);
  ref_pose.pose.position.z = reference(2, 0);
  ref_pose.pose.orientation.w = reference(3, 0);
  ref_pose.pose.orientation.x = reference(4, 0);
  ref_pose.pose.orientation.y = reference(5, 0);
  ref_pose.pose.orientation.z = reference(6, 0);
  debug_ref_pose_pub.publish(ref_pose);

  // 3. Publish Raw Control Outputs from MPC
  std_msgs::Float32MultiArray ctrl_msg;
  ctrl_msg.data.push_back(control[0]); // thrust
  ctrl_msg.data.push_back(control[1]); // wx
  ctrl_msg.data.push_back(control[2]); // wy
  ctrl_msg.data.push_back(control[3]); // wz
  debug_control_pub.publish(ctrl_msg);

  double thrust = 0;
  //thrust = control[0] * hover_thrust / 9.8066;
  //thrust = control[0] * hover_thrust / 9.8066;
  std::cout<<"hov_thrust = 9.8066 / thrust_estimator->getThr2Acc() = "<<9.8066 / thrust_estimator->getThr2Acc()<<std::endl;
  thrust = thrust_estimator->computeDesiredThrust(control[0]);

  // 仿真第一阶段安全限制
  if(thrust > 0.8)
    thrust = 0.8;

  if(thrust < 0.05)
    thrust = 0.05;

  thrust_estimator->pushThrustRecord(ros::Time::now().toSec(), thrust);
  //ROS_INFO_THROTTLE(1.0, "Thr2Acc: %f, estimated hover_thrust: %f", thrust_estimator->getThr2Acc(), 9.8066 / thrust_estimator->getThr2Acc());

  // 4. Publish Estimated Hover Thrust
  std_msgs::Float64 hover_thrust_msg;
  hover_thrust_msg.data = 9.8066 / thrust_estimator->getThr2Acc();
  debug_hover_thrust_pub.publish(hover_thrust_msg);

  mavros_msgs::AttitudeTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = std::string("FCU");
  cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  cmd.body_rate.x = control[1];
  cmd.body_rate.y = control[2];
  cmd.body_rate.z = control[3];
  cmd.thrust = thrust;

  // std::cout << "Input:\n" << thrust << std::endl 
  //                         << control[1] << std::endl 
  //                         << control[2] << std::endl 
  //                         << control[3] << std::endl;
  cmd_pub.publish(cmd);
}
