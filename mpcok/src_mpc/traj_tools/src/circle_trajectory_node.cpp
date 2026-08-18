#include <ros/ros.h>
#include <ros/topic.h>

#include <nav_msgs/Odometry.h>

#include <quadrotor_msgs/mpc_ref_point.h>
#include <quadrotor_msgs/mpc_ref_traj.h>

#include <cmath>
#include <string>

#define KSAMPLE 20
#define T_STEP 0.1

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_trajectory_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double radius;
    double speed;
    double height;
    double pub_hz;
    std::string odom_topic;

    pnh.param("radius", radius, 2.0);
    pnh.param("speed", speed, 0.5);
    pnh.param("height", height, 0.8);
    pnh.param("pub_hz", pub_hz, 100.0);
    pnh.param<std::string>(
        "odom_topic",
        odom_topic,
        "/mavros/local_position/odom");

    if(radius <= 0.0)
    {
        ROS_ERROR("radius must be > 0");
        return -1;
    }

    const double omega = speed / radius;

    ros::Publisher mpc_ref_pub =
        nh.advertise<quadrotor_msgs::mpc_ref_traj>(
            "/mpc_ref_traj", 1);

    ROS_INFO("Waiting for odometry...");

    nav_msgs::OdometryConstPtr odom =
        ros::topic::waitForMessage<nav_msgs::Odometry>(
            odom_topic,
            nh,
            ros::Duration(10.0));

    if(!odom)
    {
        ROS_ERROR("No odometry received.");
        return -1;
    }

    // 以启动轨迹节点时飞机当前位置作为圆轨迹起点
    const double x0 = odom->pose.pose.position.x;
    const double y0 = odom->pose.pose.position.y;

    /*
     * 圆心放在起点的 +Y 方向：
     *
     *       center
     *         |
     *         | R
     *         |
     *       start ---> 初始运动方向 +X
     *
     * 这样 t=0：
     * position = 当前悬停位置
     * velocity 沿 +X
     * yaw 初始约为 0
     */
    const double center_x = x0;
    const double center_y = y0 + radius;

    ROS_INFO("Circle start:  x = %.3f, y = %.3f", x0, y0);
    ROS_INFO("Circle center: x = %.3f, y = %.3f",
             center_x, center_y);
    ROS_INFO("R = %.2f m, v = %.2f m/s, omega = %.3f rad/s",
             radius, speed, omega);

    // 等待 NMPC subscriber 建立
    ros::Duration(0.5).sleep();

    const ros::Time start_time = ros::Time::now();

    ros::Rate rate(pub_hz);

    while(ros::ok())
    {
        const double t =
            (ros::Time::now() - start_time).toSec();

        quadrotor_msgs::mpc_ref_traj traj_msg;

        traj_msg.mpc_ref_points.reserve(KSAMPLE + 1);

        for(int i = 0; i <= KSAMPLE; ++i)
        {
            const double rt = t + i * T_STEP;
            const double theta = omega * rt;

            /*
             * 圆轨迹：
             *
             * x = xc + R sin(theta)
             * y = yc - R cos(theta)
             */
            const double x =
                center_x + radius * std::sin(theta);

            const double y =
                center_y - radius * std::cos(theta);

            const double vx =
                radius * omega * std::cos(theta);

            const double vy =
                radius * omega * std::sin(theta);

            const double ax =
                -radius * omega * omega * std::sin(theta);

            const double ay =
                radius * omega * omega * std::cos(theta);

            quadrotor_msgs::mpc_ref_point point;

            point.position.x = x;
            point.position.y = y;
            point.position.z = height;

            point.velocity.x = vx;
            point.velocity.y = vy;
            point.velocity.z = 0.0;

            point.acceleration.x = ax;
            point.acceleration.y = ay;
            point.acceleration.z = 0.0;

            traj_msg.mpc_ref_points.push_back(point);
        }

        /*
         * 这里故意把 goal 设置为圆心。
         *
         * 飞机沿半径 R 的圆运动，不会到达圆心，
         * 因而当前 ros_mission.cpp 不会因为
         * reachgoal() 自动退出 AUTO_TRACKING。
         */
        traj_msg.goal.x = center_x;
        traj_msg.goal.y = center_y;
        traj_msg.goal.z = height;

        mpc_ref_pub.publish(traj_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
