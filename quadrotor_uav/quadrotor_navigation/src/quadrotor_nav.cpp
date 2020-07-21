//
// Created by zhijun on 2020/6/23.
//

#include "quadrotor_nav.h"

QuadrotorNavigation::QuadrotorNavigation()
: x_(0), y_(0), z_(0)
{
    double xy_p = 1.0, xy_i = 0, xy_d = 0.2;
    double z_p = 1.0, z_i = 0.2, z_d = 0.2;
    double roll_pitch_p = 0.1, roll_pitch_i = 0, roll_pitch_d = 0.02;
    double yaw_p = 0.1, yaw_i = 0, yaw_d = 0.02;

    setXYPid(xy_p, xy_i, xy_d);
    setZPid(z_p, z_i, z_d);
    setRollPitchPid(roll_pitch_p, roll_pitch_i, roll_pitch_d);
    setYawPid(yaw_p, yaw_i, yaw_d);

    uav_wrench_pub_  = nh_.advertise<geometry_msgs::Wrench>("/uav/cmd_force", 1);

    uav_state_sub_  = nh_.subscribe("/uav/odometry", 1, &QuadrotorNavigation::uavPoseCallback, this);

    prev_time_ = ros::Time::now();

    dr_server_.setCallback(boost::bind(&QuadrotorNavigation::dynamicReconfigureCallback,this,_1,_2));

    subscribe_thread_ = std::thread(std::bind(&QuadrotorNavigation::subscribe_spin, this) );

}

QuadrotorNavigation::~QuadrotorNavigation()
{
    subscribe_thread_.join();
}

void QuadrotorNavigation::uavPoseCallback(const nav_msgs::OdometryConstPtr &pose)
{
    while (ros::Time::now().isZero() ) ; // get rid of time 0 state & make sure dt > 0
    cur_time_ = ros::Time::now();
    auto dt = cur_time_ - prev_time_;

    std::lock_guard<std::mutex> lock(uav_lock_);
    uav_x_ = pose->pose.pose.position.x;
    uav_y_ = pose->pose.pose.position.y;
    uav_z_ = pose->pose.pose.position.z;
    tf::Quaternion q(pose->pose.pose.orientation.x,
                     pose->pose.pose.orientation.y,
                     pose->pose.pose.orientation.z,
                     pose->pose.pose.orientation.w);

    tf::Matrix3x3(q).getRPY(uav_roll_, uav_pitch_, uav_yaw_);

    geometry_msgs::Wrench uav_force;
    uav_force.torque.x = uav_roll_pid_->Update(dt.toSec(), 0, uav_roll_);
    uav_force.torque.y = uav_pitch_pid_->Update(dt.toSec(), 0, uav_pitch_);
    uav_force.torque.z = uav_yaw_pid_->Update(dt.toSec(),0, uav_yaw_);

    uav_force.force.x = uav_x_pid_->Update(dt.toSec(), x_, uav_x_);
    uav_force.force.y = uav_y_pid_->Update(dt.toSec(), y_, uav_y_);
    uav_force.force.z = 14.4746 + uav_z_pid_->Update(dt.toSec(), z_, uav_z_);
    //uav_force.force.z = uav_z_pid_->Update(dt.toSec(), z_, uav_z_);

    uav_wrench_pub_.publish(uav_force);

    prev_time_ = cur_time_;
}

void QuadrotorNavigation::dynamicReconfigureCallback(FlyControlPidConfig &config, uint32_t level)
{
    std::lock_guard<std::mutex> lockGuard(pid_lock_);
    x_ = config.X;
    y_ = config.Y;
    z_ = config.Z;
    uav_x_pid_->SetKp(config.xy_p);
    uav_x_pid_->SetKi(config.xy_i);
    uav_x_pid_->SetKd(config.xy_d);

    uav_y_pid_->SetKp(config.xy_p);
    uav_y_pid_->SetKi(config.xy_i);
    uav_y_pid_->SetKd(config.xy_d);

    uav_z_pid_->SetKp(config.z_p);
    uav_z_pid_->SetKi(config.z_i);
    uav_z_pid_->SetKd(config.z_d);

    uav_roll_pid_->SetKp(config.rp_p);
    uav_roll_pid_->SetKi(config.rp_i);
    uav_roll_pid_->SetKd(config.rp_d);

    uav_pitch_pid_->SetKp(config.rp_p);
    uav_pitch_pid_->SetKi(config.rp_i);
    uav_pitch_pid_->SetKd(config.rp_d);

    uav_yaw_pid_->SetKp(config.yaw_p);
    uav_yaw_pid_->SetKi(config.yaw_i);
    uav_yaw_pid_->SetKd(config.yaw_d);

    ROS_INFO("Reconfigure Results : \n XY: %f, %f, %f \n Z: %f, %f, %f \n RP: %f, %f, %f \n Yaw: %f, %f, %f \n X: %f, Y: %f, Z: %f",
             uav_x_pid_->GetKp(), uav_x_pid_->GetKi(), uav_x_pid_->GetKd(),
             uav_z_pid_->GetKp(), uav_z_pid_->GetKi(), uav_z_pid_->GetKd(),
             uav_roll_pid_->GetKp(), uav_roll_pid_->GetKi(), uav_roll_pid_->GetKd(),
             uav_yaw_pid_->GetKp(), uav_yaw_pid_->GetKi(), uav_yaw_pid_->GetKd(),
             x_, y_, z_);
}

void QuadrotorNavigation::subscribe_spin()
{
    static const double timeout = 0.0;

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void QuadrotorNavigation::setXYPid(const double & p, const double & i, const double & d)
{
    uav_x_pid_.reset(new Pid<double>(xy_ctrl_max_, -xy_ctrl_max_, xy_ctrl_max_, -xy_ctrl_max_, p, i, d));
    uav_y_pid_.reset(new Pid<double>(xy_ctrl_max_, -xy_ctrl_max_, xy_ctrl_max_, -xy_ctrl_max_, p, i, d));
}

void QuadrotorNavigation::setZPid(const double & p, const double & i, const double & d)
{
    uav_z_pid_.reset(new Pid<double>(z_ctrl_max_, -z_ctrl_max_, z_ctrl_max_, -z_ctrl_max_, p, i, d));
}

void QuadrotorNavigation::setRollPitchPid(const double & p, const double & i, const double & d)
{
    uav_roll_pid_.reset(new Pid<double>(roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_, roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_, p, i, d));
    uav_pitch_pid_.reset(new Pid<double>(roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_, roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_, p, i, d));
}

void QuadrotorNavigation::setYawPid(const double & p, const double & i, const double & d)
{
    uav_yaw_pid_.reset(new Pid<double>(yaw_ctrl_max_, -yaw_ctrl_max_, yaw_ctrl_max_, -yaw_ctrl_max_, p, i, d));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quadrotor_nav");

    QuadrotorNavigation qn;

    return 0;
}