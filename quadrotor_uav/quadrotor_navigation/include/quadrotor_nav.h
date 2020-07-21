//
// Created by zhijun on 2020/6/23.
//

#ifndef SRC_QUADROTOR_NAV_H
#define SRC_QUADROTOR_NAV_H

#include <thread>
#include <mutex>
#include <iostream>
#include <functional>
#include <memory>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <quadrotor_navigation/FlyControlPidConfig.h>

#include "pid.h"

class QuadrotorNavigation
{
public:
    QuadrotorNavigation();
    ~QuadrotorNavigation();


    void setXYPid(const double & p, const double & i, const double & d);
    void setZPid(const double & p, const double & i, const double & d);
    void setRollPitchPid(const double & p, const double & i, const double & d);
    void setYawPid(const double & p, const double & i, const double & d);

    typedef quadrotor_navigation::FlyControlPidConfig FlyControlPidConfig;

private:
    void dynamicReconfigureCallback(FlyControlPidConfig &config, uint32_t level);

    void uavPoseCallback(const nav_msgs::OdometryConstPtr & pose);

    void subscribe_spin();

    ros::NodeHandle nh_;
    ros::Publisher  uav_wrench_pub_;
    ros::Subscriber uav_state_sub_;

    // set position
    double x_, y_, z_;

    // UAV pose
    double uav_x_, uav_y_, uav_z_;
    tfScalar uav_roll_, uav_pitch_, uav_yaw_;

    dynamic_reconfigure::Server<FlyControlPidConfig> dr_server_;

    // pid for UAV position control
    std::shared_ptr<Pid<double> > uav_x_pid_;
    std::shared_ptr<Pid<double> > uav_y_pid_;
    std::shared_ptr<Pid<double> > uav_z_pid_;
    std::shared_ptr<Pid<double> > uav_roll_pid_;
    std::shared_ptr<Pid<double> > uav_pitch_pid_;
    std::shared_ptr<Pid<double> > uav_yaw_pid_;

    std::mutex pid_lock_;
    std::mutex uav_lock_;
    std::thread subscribe_thread_;

    ros::Time prev_time_;
    ros::Time cur_time_;

    constexpr static double xy_ctrl_max_ = 3;
    constexpr static double z_ctrl_max_ = 3;
    constexpr static double roll_pitch_ctrl_max_ = 0.03;
    constexpr static double yaw_ctrl_max_ = 0.05;

};

#endif //SRC_QUADROTOR_NAV_H
