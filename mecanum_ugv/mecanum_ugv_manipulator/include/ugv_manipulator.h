//
// Created by zhijun on 2020/6/27.
//

#ifndef SRC_UGV_MANIPULATOR_H
#define SRC_UGV_MANIPULATOR_H

#include <thread>
#include <mutex>
#include <iostream>
#include <functional>
#include <memory>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <mecanum_ugv_manipulator/UgvManipulatorPidConfig.h>

#include "pid.h"

class UgvManipulator
{
public:
    UgvManipulator();
    ~UgvManipulator();

    typedef mecanum_ugv_manipulator::UgvManipulatorPidConfig UgvManipulatorPidConfig;

    void dynamicReconfigCallback(UgvManipulatorPidConfig & config, uint32_t level);

private:
    void ugvTwistCmdCallback(const geometry_msgs::Twist::ConstPtr & twist);
    void ugvOdomCallback(const nav_msgs::Odometry::ConstPtr & odom);

    void subscribeSpin();

    ros::NodeHandle nh_;
    ros::Subscriber ugv_twist_sub_;
    ros::Publisher  ugv_wrench_pub_;

    ros::NodeHandle nh2_;
    ros::Subscriber ugv_odom_sub_;
    ros::CallbackQueue ugv_queue_;

    dynamic_reconfigure::Server<UgvManipulatorPidConfig> server_;

    // pid for UAV position control
    std::shared_ptr<Pid<double> > ugv_x_pid_;
    std::shared_ptr<Pid<double> > ugv_side_pid_;
    std::shared_ptr<Pid<double> > ugv_rot_pid_;

    ros::Time prev_time_;
    ros::Time cur_time_;
    std::thread subscribe_thread_;

    std::mutex ugv_lock_;
    double cmd_vel_x_, cmd_vel_y_, cmd_rot_z_;
};

#endif //SRC_UGV_MANIPULATOR_H
