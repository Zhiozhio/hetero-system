//
// Created by zhijun on 2020/6/27.
//

#include "ugv_manipulator.h"

UgvManipulator::UgvManipulator()
: cmd_vel_x_(0), cmd_vel_y_(0), cmd_rot_z_(0)
{
    ugv_x_pid_ = std::make_shared<Pid<double>>(500,-500,20000,-20000,1000,0,0);
    ugv_side_pid_ = std::make_shared<Pid<double>>(500,-500,20000,-20000,1000,0,0);
    ugv_rot_pid_ = std::make_shared<Pid<double>>(100,-100,2000,-2000,200,0,0);

    ugv_twist_sub_ = nh_.subscribe("/ugv/cmd_vel", 1, &UgvManipulator::ugvTwistCmdCallback, this);

    ugv_wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("/ugv/cmd_force", 1);

    nh2_.setCallbackQueue(&ugv_queue_);
    ugv_odom_sub_ = nh2_.subscribe("/ugv/odometry", 1, &UgvManipulator::ugvOdomCallback, this);

    prev_time_ = ros::Time::now();

    //server_ = new dynamic_reconfigure::Server<UgvManipulatorPidConfig>(nh_);
    dynamic_reconfigure::Server<UgvManipulatorPidConfig>::CallbackType f;
    f = boost::bind(&UgvManipulator::dynamicReconfigCallback, this, _1, _2);

    server_.setCallback(f);

    subscribe_thread_ = std::thread(std::bind(&UgvManipulator::subscribeSpin, this) );
}

UgvManipulator::~UgvManipulator()
{
    subscribe_thread_.join();
}

void UgvManipulator::ugvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    while (ros::Time::now().isZero() ) ; // get rid of time 0 state & make sure dt > 0
    cur_time_ = ros::Time::now();
    auto dt = cur_time_ - prev_time_;
    if (dt.isZero()) return ;

    // read current velocity states
    double vel_x = odom->twist.twist.linear.x;
    double vel_y = odom->twist.twist.linear.y;
    double rot_z = odom->twist.twist.angular.z;

    // read current command values
    double cmd_vx = cmd_vel_x_;
    double cmd_vy = cmd_vel_y_;
    double cmd_rz = cmd_rot_z_;

    geometry_msgs::Wrench wrench;

    wrench.force.x = ugv_x_pid_->Update(dt.toSec(), cmd_vx, vel_x);
    wrench.force.y = ugv_side_pid_->Update(dt.toSec(), cmd_vy, vel_y);
    wrench.force.z = 0;
    wrench.torque.x = 0;
    wrench.torque.y = 0;
    wrench.torque.z = ugv_rot_pid_->Update(dt.toSec(), cmd_rz, rot_z);

    ugv_wrench_pub_.publish(wrench);
    prev_time_ = cur_time_;
}

void UgvManipulator::ugvTwistCmdCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
    // read the command values
    std::lock_guard<std::mutex> lock(ugv_lock_);
    cmd_vel_x_ = twist->linear.x;
    cmd_vel_y_ = twist->linear.y;
    cmd_rot_z_ = twist->angular.z;
}

void UgvManipulator::dynamicReconfigCallback(UgvManipulatorPidConfig & config, uint32_t level)
{
    ugv_x_pid_->SetKp(config.x_p);
    ugv_x_pid_->SetKi(config.x_i);
    ugv_x_pid_->SetKd(config.x_d);

    ugv_side_pid_->SetKp(config.y_p);
    ugv_side_pid_->SetKi(config.y_i);
    ugv_side_pid_->SetKd(config.y_d);

    ugv_rot_pid_->SetKp(config.rot_p);
    ugv_rot_pid_->SetKi(config.rot_i);
    ugv_rot_pid_->SetKd(config.rot_d);

    ROS_INFO("Reconfigure Results : \n X: %f, %f, %f \n Y: %f, %f, %f \n Rot: %f, %f, %f",
             ugv_x_pid_->GetKp(), ugv_x_pid_->GetKi(), ugv_x_pid_->GetKd(),
             ugv_side_pid_->GetKp(), ugv_side_pid_->GetKi(), ugv_side_pid_->GetKd(),
             ugv_rot_pid_->GetKp(), ugv_rot_pid_->GetKi(), ugv_rot_pid_->GetKd()
    );


}

void UgvManipulator::subscribeSpin()
{
    static const double timeout = 0.0;

    ros::Rate loop_rate(20);
    while (ros::ok() )
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(timeout));
        ugv_queue_.callAvailable(ros::WallDuration(timeout));
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ugv_manipulator");

    UgvManipulator um;

    return 0;
}
