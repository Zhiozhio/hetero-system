//
// Created by zhijun on 2020/6/17.
//

#include "uav_ugv_bridge.h"
#include "pid.h"

UavUgvBridge::UavUgvBridge()
: ugv_stop_begin_(-1), ugv_move_(false), uav_landed_(false), prev_set_(false), taking_off_(false),
step_height_(0.0), step_begin_(-1)
{
    // this parameters are tuned in the FlyControlPid.cfg in quadrotor_navigation
    uav_x_pid_.reset(new Pid<double>(xy_ctrl_max_, -xy_ctrl_max_, xy_ctrl_max_, -xy_ctrl_max_, 3, 0, 3.8));
    uav_y_pid_.reset(new Pid<double>(xy_ctrl_max_, -xy_ctrl_max_, xy_ctrl_max_, -xy_ctrl_max_, 3, 0, 3.8));
    uav_z_pid_.reset(new Pid<double>(z_ctrl_max_, -z_ctrl_max_, z_ctrl_max_, -z_ctrl_max_, 3, 0, 4.5));

    uav_roll_pid_.reset(new Pid<double>(
            roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_, roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_,
            0.5, 0, 0.2)
            );
    uav_pitch_pid_.reset(new Pid<double>(
            roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_, roll_pitch_ctrl_max_, -roll_pitch_ctrl_max_,
            0.5, 0, 0.2)
            );
    uav_yaw_pid_.reset(new Pid<double>(
            yaw_ctrl_max_, -yaw_ctrl_max_, yaw_ctrl_max_, -yaw_ctrl_max_, 0.7, 0, 0.3)
            );

    ugv_state_sub_  = nh_.subscribe("/ugv/pose", 1, &UavUgvBridge::ugvPoseCallback, this);

    uav_wrench_pub_  = nh_.advertise<geometry_msgs::Wrench>("/uav/cmd_force", 1);

    // use another nodehandle to bind to alternative callback queue
    nh2_.setCallbackQueue(&uav_queue_);
    uav_state_sub_  = nh2_.subscribe("/uav/pose", 1, &UavUgvBridge::uavPoseCallback, this);

    //takeoff_server_ = nh2_.advertiseService("uav_take_off", &UavUgvBridge::takeoffServerCallback, this);

    subscribe_thread_ = std::thread(std::bind(&UavUgvBridge::subscribe_spin, this) );

}

UavUgvBridge::~UavUgvBridge()
{
    subscribe_thread_.join();
}

void UavUgvBridge::ugvPoseCallback(const nav_msgs::OdometryConstPtr & pose)
{
    std::lock_guard<std::mutex> lock(ugv_lock_);
    ugv_pose_.position.x = pose->pose.pose.position.x;
    ugv_pose_.position.y = pose->pose.pose.position.y;
    ugv_pose_.position.z = pose->pose.pose.position.z;

    if (!ros::Time::now().isZero() && !prev_set_)  // get rid of time 0 state
    {
        prev_x_ = ugv_pose_.position.x;
        prev_y_ = ugv_pose_.position.y;
        prev_time_ = ros::Time::now();
        prev_set_ = true;
    }
}

void UavUgvBridge::uavPoseCallback(const nav_msgs::OdometryConstPtr & pose)
{
    if (!prev_set_ ) return ;

    cur_time_ = ros::Time::now();
    auto dt = cur_time_ - prev_time_;
    if (dt.isZero()) return ; // this may happen at the first time

    // process the subscribed message
    std::lock_guard<std::mutex> lock(uav_lock_);
    uav_x_ = pose->pose.pose.position.x;
    uav_y_ = pose->pose.pose.position.y;
    uav_z_ = pose->pose.pose.position.z;
    tf::Quaternion q(pose->pose.pose.orientation.x,
                     pose->pose.pose.orientation.y,
                     pose->pose.pose.orientation.z,
                     pose->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(uav_roll_, uav_pitch_, uav_yaw_);

    // define current target point
    double x = ugv_pose_.position.x;
    double y = ugv_pose_.position.y;
    double z = ugv_pose_.position.z;

    if (ros::param::get("/uav_hover_height", hovering_height_) )
        ROS_INFO("hovering_height: %f", hovering_height_);
    ros::param::get("/uav_take_off", taking_off_);
    if (taking_off_)
        takeOff(dt, x, y, z);
    else
    {
        uavForcePublish(dt, x, y, z);
    }
}

void UavUgvBridge::uavForcePublish(const ros::Duration &dt, const double &x, const double &y, const double &z)
{
    // check if UGV is moving
    ugv_move_ = (x-prev_x_)*(x-prev_x_) + (y-prev_y_)*(y-prev_y_) > ugv_move_thresh_ * dt.toSec();
    // check if UAV is landed
    uav_landed_ = uav_z_ <= z + uav_landed_height_thresh_;

    // define the message that would be published
    geometry_msgs::Wrench uav_force;
    uav_force.force.x = uav_force.force.y = uav_force.force.z = 0;
    uav_force.torque.x = uav_force.torque.y = uav_force.torque.z = 0;

    if (ugv_move_)
    {
        write_wrench(uav_force, z+hovering_height_, dt, x, y);

        ugv_stop_begin_ = -1; // stop calculate ugv static time duration
    }
    else
    {
        if (ugv_stop_begin_ < 0.0) ugv_stop_begin_ = ros::Time::now().toSec(); // ugv begin to stop from motion

        if (uav_landed_) ; // now has landed and no force command

        else // uav not landed
        {
            if (ros::Time::now().toSec() - ugv_stop_begin_ > stay_hovering_time_)
            {
                // landing
                write_wrench(uav_force, z+uav_landed_height_thresh_, dt, x, y);
            }
            else
            {
                // still tracking
                write_wrench(uav_force, z+hovering_height_, dt, x, y);
            }
        }
    }
    uav_wrench_pub_.publish(uav_force);

    prev_x_ = x;
    prev_y_ = y;
    prev_time_ = cur_time_;

}



void UavUgvBridge::takeOff(const ros::Duration &dt, const double &x, const double &y, const double &z)
{
    if (uav_z_ >= z+hovering_height_)
    {
        ros::param::set("/uav_take_off", false);
        step_begin_ = -1;
        step_height_ = 0;
        ugv_stop_begin_ = -1; // we want uav hover 5 secs too

        return ;
    }

    if (ros::param::get("/uav_take_off/step_length", step_interval_) )
        ROS_INFO("step_interval: %f", step_interval_);
    if (ros::param::get("/uav_take_off/step_time", step_time_) )
        ROS_INFO("step_time: %f", step_time_);
    if (step_begin_ == -1 || cur_time_ - ros::Time(step_begin_) >= ros::Duration(step_time_) )
    {
        step_begin_ = cur_time_.toSec();
        step_height_ += step_interval_;
    }

    // define the message that would be published
    geometry_msgs::Wrench uav_force;
    uav_force.force.x = uav_force.force.y = uav_force.force.z = 0;
    uav_force.torque.x = uav_force.torque.y = uav_force.torque.z = 0;

    write_wrench(uav_force, z+step_height_, dt, x, y);
    uav_wrench_pub_.publish(uav_force);

    prev_time_ = cur_time_;

}

///\brief expected coordinate: x, y, target_height
/// wrench publish interval: dt
void UavUgvBridge::write_wrench(geometry_msgs::Wrench &wrench, const double &target_height, const ros::Duration &dt,
                                const double &x, const double &y)
{
    // fill in the roll pitch yaw torque
    wrench.torque.x = uav_roll_pid_->Update(dt.toSec(), 0, uav_roll_);
    wrench.torque.y = uav_pitch_pid_->Update(dt.toSec(), 0, uav_pitch_);
    wrench.torque.z = uav_yaw_pid_->Update(dt.toSec(),0, uav_yaw_);

    wrench.force.x = uav_x_pid_->Update(dt.toSec(), x, uav_x_);
    wrench.force.y = uav_y_pid_->Update(dt.toSec(), y, uav_y_);
    wrench.force.z = UAV_WEIGHT + uav_z_pid_->Update(dt.toSec(), target_height, uav_z_);
}

void UavUgvBridge::subscribe_spin()
{
    static const double timeout = 0.0;

    ros::Rate loop_rate(20);
    while (ros::ok() )
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(timeout));
        uav_queue_.callAvailable(ros::WallDuration(timeout));
        loop_rate.sleep();
    }
}
