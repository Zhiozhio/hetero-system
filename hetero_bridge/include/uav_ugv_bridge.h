//
// Created by zhijun on 2020/6/16.
// UavUgvBridge subscribes the UGV & UAV position info and process it,
// then publishes command force to UAV to following the UGV.
//

#ifndef SRC_UAV_UGV_BRIDGE_H
#define SRC_UAV_UGV_BRIDGE_H

#define UAV_WEIGHT 14.4746

#include <thread>
#include <mutex>
#include <iostream>
#include <functional>
#include <memory>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>

//#include "hetero_bridge/TakeOff.h"
#include "pid.h"

class UavUgvBridge
{
public:
    UavUgvBridge();
    ~UavUgvBridge();


private:
    // void ugvPoseCallback(const geometry_msgs::Pose::ConstPtr & pose);
    // void uavPoseCallback(const geometry_msgs::Pose::ConstPtr & pose);
    void ugvPoseCallback(const nav_msgs::OdometryConstPtr & pose);
    void uavPoseCallback(const nav_msgs::OdometryConstPtr & pose);

    void uavForcePublish(const ros::Duration & dt, const double& x, const double& y, const double& z);

    //bool takeoffServerCallback(hetero_bridge::TakeOff::Request &req, hetero_bridge::TakeOff::Response &res);

    void takeOff(const ros::Duration & dt, const double& x, const double& y, const double &z);

    void write_wrench(geometry_msgs::Wrench &wrench, const double &target_height, const ros::Duration &dt,
                      const double& x, const double& y);

    void subscribe_spin();

    ros::NodeHandle nh_;
    ros::Subscriber ugv_state_sub_;
    ros::Publisher  uav_wrench_pub_;

    ros::NodeHandle nh2_;
    ros::Subscriber uav_state_sub_;
    //ros::ServiceServer takeoff_server_;

    // UGV pose
    geometry_msgs::Pose ugv_pose_;

    // UAV pose
    tfScalar uav_x_, uav_y_, uav_z_;
    tfScalar uav_roll_, uav_pitch_, uav_yaw_;

    // pid for UAV position control
    std::shared_ptr<Pid<double> > uav_x_pid_;
    std::shared_ptr<Pid<double> > uav_y_pid_;
    std::shared_ptr<Pid<double> > uav_z_pid_;
    std::shared_ptr<Pid<double> > uav_roll_pid_;
    std::shared_ptr<Pid<double> > uav_pitch_pid_;
    std::shared_ptr<Pid<double> > uav_yaw_pid_;

    std::mutex ugv_lock_;
    std::mutex uav_lock_;
    std::thread subscribe_thread_;

    ros::CallbackQueue uav_queue_;

    ros::Time prev_time_;
    ros::Time cur_time_;
    // previous target point
    double prev_x_, prev_y_;
    bool prev_set_;

    constexpr static double ugv_move_thresh_ = 0.001;
    constexpr static double uav_landed_height_thresh_ = 0.58; // 0.35+0.23
    constexpr static double stay_hovering_time_ = 5.0;

    double hovering_height_ = 3.0;

    // this parameters are tuned in the FlyControlPid.cfg in quadrotor_navigation
    constexpr static double xy_ctrl_max_ = 3;
    constexpr static double z_ctrl_max_ = 3;
    constexpr static double roll_pitch_ctrl_max_ = 0.03;
    constexpr static double yaw_ctrl_max_ = 0.05;

    bool ugv_move_;
    bool uav_landed_;
    double ugv_stop_begin_;

    // take off initialization fields
    bool taking_off_;
    double step_begin_;
    //ros::Time step_begin_time_;
    double step_height_;
    double step_interval_ = 0.25;
    double step_time_ = 0.55;
};

#endif //SRC_UAV_UGV_BRIDGE_H
