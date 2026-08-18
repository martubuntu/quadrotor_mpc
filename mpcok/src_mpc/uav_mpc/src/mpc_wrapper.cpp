#include "uav_mpc/mpc_wrapper.h"

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MPCWrapper::MPCWrapper(ros::NodeHandle &nh):nh(nh)
{
  nh.getParam("/cost/cost_px", cost_px); 
  nh.getParam("/cost/cost_py", cost_py); 
  nh.getParam("/cost/cost_pz", cost_pz); 
  nh.getParam("/cost/cost_qw", cost_qw); 
  nh.getParam("/cost/cost_qx", cost_qx); 
  nh.getParam("/cost/cost_qy", cost_qy); 
  nh.getParam("/cost/cost_qz", cost_qz); 
  nh.getParam("/cost/cost_vx", cost_vx); 
  nh.getParam("/cost/cost_vy", cost_vy); 
  nh.getParam("/cost/cost_vz", cost_vz); 
  nh.getParam("/cost/cost_thrust", cost_thrust); 
  nh.getParam("/cost/cost_wx", cost_wx); 
  nh.getParam("/cost/cost_wy", cost_wy); 
  nh.getParam("/cost/cost_wz", cost_wz); 
  nh.getParam("/boudings/T_max", T_max); 
  nh.getParam("/boudings/T_min", T_min); 
  nh.getParam("/boudings/wx_max", wx_max); 
  nh.getParam("/boudings/wx_min", wx_min); 
  nh.getParam("/boudings/wy_max", wy_max); 
  nh.getParam("/boudings/wy_min", wy_min); 
  nh.getParam("/boudings/wz_max", wz_max); 
  nh.getParam("/boudings/wz_min", wz_min); 

  pub_pred_path = nh.advertise<nav_msgs::Path>("/mpc_debug/acado_pred_path", 1);
  pub_ref_path = nh.advertise<nav_msgs::Path>("/mpc_debug/acado_ref_path", 1);
  pub_pred_u = nh.advertise<std_msgs::Float32MultiArray>("/mpc_debug/acado_pred_u", 1);
  pub_x0 = nh.advertise<std_msgs::Float32MultiArray>("/mpc_debug/acado_x0", 1);

  disturbance_.setZero();

  eso_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(
      "/eso/disturbance",
      10,
      &MPCWrapper::esoCallback,
      this);
}

MPCWrapper::~MPCWrapper()
{
}

bool MPCWrapper::initSolver(nav_msgs::Odometry& msg)
{
    Eigen::Quaterniond q_init(
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z);

    q_init.normalize();

    Eigen::Vector3d v_body_init(
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z);

    // base_link/body -> map/world
    Eigen::Vector3d v_world_init = q_init * v_body_init;
  /* Clear solver memory. */
  memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  memset(&acadoVariables, 0, sizeof( acadoVariables ));

  /* Initialize the solver. */
  acado_initializeSolver();

  /* Initialize the states and controls. */
  for (int i = 0; i < N + 1; ++i)
  {
	  acadoVariables.x[i * NX + 0] = msg.pose.pose.position.x;
	  acadoVariables.x[i * NX + 1] = msg.pose.pose.position.y;
	  acadoVariables.x[i * NX + 2] = msg.pose.pose.position.z;
	  acadoVariables.x[i * NX + 3] = msg.pose.pose.orientation.w;
	  acadoVariables.x[i * NX + 4] = msg.pose.pose.orientation.x;
	  acadoVariables.x[i * NX + 5] = msg.pose.pose.orientation.y;
	  acadoVariables.x[i * NX + 6] = msg.pose.pose.orientation.z;
          acadoVariables.x[i * NX + 7] = v_world_init.x();
          acadoVariables.x[i * NX + 8] = v_world_init.y();
          acadoVariables.x[i * NX + 9] = v_world_init.z();
  }

  for (int i = 0; i < NX; ++i)
	  acadoVariables.x0[ i ] = acadoVariables.x[ i ];

  /* Initialize the Cost Matrix. */
  for(int i = 0; i < N; ++i)
  {
	  acadoVariables.W[ i * (NY * NY) + 0 * NY + 0] = cost_px;
	  acadoVariables.W[ i * (NY * NY) + 1 * NY + 1] = cost_py;
    acadoVariables.W[ i * (NY * NY) + 2 * NY + 2] = cost_pz;
	  acadoVariables.W[ i * (NY * NY) + 3 * NY + 3] = cost_qw;
	  acadoVariables.W[ i * (NY * NY) + 4 * NY + 4] = cost_qx;
	  acadoVariables.W[ i * (NY * NY) + 5 * NY + 5] = cost_qy;
	  acadoVariables.W[ i * (NY * NY) + 6 * NY + 6] = cost_qz;
	  acadoVariables.W[ i * (NY * NY) + 7 * NY + 7] = cost_vx;
	  acadoVariables.W[ i * (NY * NY) + 8 * NY + 8] = cost_vy;
	  acadoVariables.W[ i * (NY * NY) + 9 * NY + 9] = cost_vz;
	  acadoVariables.W[ i * (NY * NY) + 10 * NY + 10] = cost_thrust;
	  acadoVariables.W[ i * (NY * NY) + 11 * NY + 11] = cost_wx;
	  acadoVariables.W[ i * (NY * NY) + 12 * NY + 12] = cost_wy;
	  acadoVariables.W[ i * (NY * NY) + 13 * NY + 13] = cost_wz;
  }

  acadoVariables.WN[0 * NX + 0] = cost_px;
  acadoVariables.WN[1 * NX + 1] = cost_py;
  acadoVariables.WN[2 * NX + 2] = cost_pz;
  acadoVariables.WN[3 * NX + 3] = cost_qw;
  acadoVariables.WN[4 * NX + 4] = cost_qx;
  acadoVariables.WN[5 * NX + 5] = cost_qy;
  acadoVariables.WN[6 * NX + 6] = cost_qz;
  acadoVariables.WN[7 * NX + 7] = cost_vx;
  acadoVariables.WN[8 * NX + 8] = cost_vy;
  acadoVariables.WN[9 * NX + 9] = cost_vz;


  /* Initialize the Boundings. */
  for(int i = 0; i < N; ++i)
  {
    // std::cout << "max" << acadoVariables.ubValues[i] << std::endl;
    // std::cout << "min" << acadoVariables.lbValues[i] << std::endl;
	  acadoVariables.ubValues[i * NU + 0] = 12;
    acadoVariables.lbValues[i * NU + 0] = 2;
    acadoVariables.ubValues[i * NU + 1] = 1.5;
    acadoVariables.lbValues[i * NU + 1] = -1.5;
    acadoVariables.ubValues[i * NU + 2] = 1.5;
    acadoVariables.lbValues[i * NU + 2] = -1.5;
    acadoVariables.ubValues[i * NU + 3] = 1;
    acadoVariables.lbValues[i * NU + 3] = -1;
  }

  /* Prepare first step */
  acado_preparationStep();

  return true;

}

void MPCWrapper::gerReference(const Eigen::MatrixXd& ref)
{
  /* Initialize the measurements/reference. */
  for (int i = 0; i < N; ++i)
  {
    acadoVariables.y[i * NY + 0] = ref.col(i)[0];      // px
	  acadoVariables.y[i * NY + 1] = ref.col(i)[1];      // py
	  acadoVariables.y[i * NY + 2] = ref.col(i)[2];      // pz
	  acadoVariables.y[i * NY + 3] = ref.col(i)[3];      // qw
	  acadoVariables.y[i * NY + 4] = ref.col(i)[4];      // qx
	  acadoVariables.y[i * NY + 5] = ref.col(i)[5];      // qy
	  acadoVariables.y[i * NY + 6] = ref.col(i)[6];      // qz
	  acadoVariables.y[i * NY + 7] = ref.col(i)[7];      // vx
	  acadoVariables.y[i * NY + 8] = ref.col(i)[8];      // vy
	  acadoVariables.y[i * NY + 9] = ref.col(i)[9];      // vz
	  acadoVariables.y[i * NY + 10] = ref.col(i)[10];    // thrust
	  acadoVariables.y[i * NY + 11] = ref.col(i)[11];    // wx
	  acadoVariables.y[i * NY + 12] = ref.col(i)[12];    // wy
	  acadoVariables.y[i * NY + 13] = ref.col(i)[13];    // wz
  }  

  for (int i = 0; i < NYN; ++i)
  {
    acadoVariables.yN[ i ] = ref.col(N)[i];
  }  
}

bool MPCWrapper::getSolution(
    nav_msgs::Odometry& msg,
    Eigen::Vector4f& control)
{
    acado_tic(&t);

    // 最新状态
    updateState(msg);

    // 最新ESO扰动
    updateOnlineData();

    status = acado_feedbackStep();

    if(status)
        return false;

    publishDebugData();

    real_t *U = acado_getVariablesU();

    control[0] = U[0];
    control[1] = U[1];
    control[2] = U[2];
    control[3] = U[3];

    acado_preparationStep();

    return true;
}


void MPCWrapper::publishDebugData()
{
  ros::Time now = ros::Time::now();

  // 1. Publish x0 (Current State Input to MPC)
  std_msgs::Float32MultiArray msg_x0;
  for(int i = 0; i < NX; ++i) msg_x0.data.push_back(acadoVariables.x0[i]);
  pub_x0.publish(msg_x0);

  // 2. Publish Predicted Path (acadoVariables.x)
  nav_msgs::Path pred_path;
  pred_path.header.stamp = now;
  pred_path.header.frame_id = "world";
  for(int i = 0; i <= N; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "world";
    pose.pose.position.x = acadoVariables.x[i * NX + 0];
    pose.pose.position.y = acadoVariables.x[i * NX + 1];
    pose.pose.position.z = acadoVariables.x[i * NX + 2];
    pose.pose.orientation.w = acadoVariables.x[i * NX + 3];
    pose.pose.orientation.x = acadoVariables.x[i * NX + 4];
    pose.pose.orientation.y = acadoVariables.x[i * NX + 5];
    pose.pose.orientation.z = acadoVariables.x[i * NX + 6];
    pred_path.poses.push_back(pose);
  }
  pub_pred_path.publish(pred_path);

  // 3. Publish Reference Path (acadoVariables.y and yN)
  nav_msgs::Path ref_path;
  ref_path.header.stamp = now;
  ref_path.header.frame_id = "world";
  for(int i = 0; i < N; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "world";
    pose.pose.position.x = acadoVariables.y[i * NY + 0];
    pose.pose.position.y = acadoVariables.y[i * NY + 1];
    pose.pose.position.z = acadoVariables.y[i * NY + 2];
    pose.pose.orientation.w = acadoVariables.y[i * NY + 3];
    pose.pose.orientation.x = acadoVariables.y[i * NY + 4];
    pose.pose.orientation.y = acadoVariables.y[i * NY + 5];
    pose.pose.orientation.z = acadoVariables.y[i * NY + 6];
    ref_path.poses.push_back(pose);
  }
  geometry_msgs::PoseStamped poseN;
  poseN.header.stamp = now;
  poseN.header.frame_id = "world";
  poseN.pose.position.x = acadoVariables.yN[0];
  poseN.pose.position.y = acadoVariables.yN[1];
  poseN.pose.position.z = acadoVariables.yN[2];
  poseN.pose.orientation.w = acadoVariables.yN[3];
  poseN.pose.orientation.x = acadoVariables.yN[4];
  poseN.pose.orientation.y = acadoVariables.yN[5];
  poseN.pose.orientation.z = acadoVariables.yN[6];
  ref_path.poses.push_back(poseN);
  pub_ref_path.publish(ref_path);

  // 4. Publish Predicted Controls (acadoVariables.u)
  std_msgs::Float32MultiArray msg_u;
  for(int i = 0; i < N * NU; ++i) msg_u.data.push_back(acadoVariables.u[i]);
  pub_pred_u.publish(msg_u);
}

void MPCWrapper::updateState(nav_msgs::Odometry& msg)
{
  acadoVariables.x0[0] = msg.pose.pose.position.x;
  acadoVariables.x0[1] = msg.pose.pose.position.y;
  acadoVariables.x0[2] = msg.pose.pose.position.z;

  Eigen::Quaterniond q(
      msg.pose.pose.orientation.w,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z);

  q.normalize();

  // 当前姿态
  Eigen::Vector4d q_current;
  q_current << q.w(), q.x(), q.y(), q.z();

  // MPC 第0节点参考四元数
  Eigen::Vector4d q_ref;
  q_ref << acadoVariables.y[3],
           acadoVariables.y[4],
           acadoVariables.y[5],
           acadoVariables.y[6];

  // q 与 -q 是同一个物理姿态。
  // 保证状态四元数和参考四元数处于同一半球。
  if(q_current.dot(q_ref) < 0.0)
  {
    q_current = -q_current;
  }

  acadoVariables.x0[3] = q_current[0];
  acadoVariables.x0[4] = q_current[1];
  acadoVariables.x0[5] = q_current[2];
  acadoVariables.x0[6] = q_current[3];

  // MAVROS odom twist: base_link/body frame
  Eigen::Vector3d v_body(
      msg.twist.twist.linear.x,
      msg.twist.twist.linear.y,
      msg.twist.twist.linear.z);

  // 转成 NMPC 所使用的 world/map frame
  Eigen::Vector3d v_world = q * v_body;

  acadoVariables.x0[7] = v_world.x();
  acadoVariables.x0[8] = v_world.y();
  acadoVariables.x0[9] = v_world.z();
}

void MPCWrapper::esoCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    disturbance_[0] = msg->vector.x;
    disturbance_[1] = msg->vector.y;
    disturbance_[2] = msg->vector.z;

    disturbance_stamp_ = ros::Time::now();
    disturbance_received_ = true;
    ROS_WARN_THROTTLE(
        1.0,
        "ESO CALLBACK RECEIVED = [%.3f %.3f %.3f]",
        disturbance_[0],
        disturbance_[1],
        disturbance_[2]);
}

void MPCWrapper::updateOnlineData()
{
    Eigen::Vector3d d = Eigen::Vector3d::Zero();

    if(disturbance_received_)
    {
        double age =
            (ros::Time::now() - disturbance_stamp_).toSec();

        if(age >= 0.0 && age < 0.2)
        {
            d = disturbance_;
        }
    }

    for(int i = 0; i <= N; ++i)
    {
        for(int j = 0; j < NOD; ++j)
        {
            acadoVariables.od[i * NOD + j] = 0.0;
        }

        acadoVariables.od[i * NOD + 0] = d.x();
        acadoVariables.od[i * NOD + 1] = d.y();
        acadoVariables.od[i * NOD + 2] = d.z();
    }

    ROS_INFO_THROTTLE(
        1.0,
        "ACADO OD = [%.3f %.3f %.3f]",
        d.x(),
        d.y(),
        d.z());
    // 就放这里，不是在 for 里面
    static int od_debug_counter = 0;
    od_debug_counter++;

    if(od_debug_counter % 100 == 0)
    {
        if(d.norm() > 0.1)
        {
            ROS_WARN(
                "===== OD ACTIVE: [%.3f %.3f %.3f] =====",
                d.x(), d.y(), d.z());
        }
    }
}
