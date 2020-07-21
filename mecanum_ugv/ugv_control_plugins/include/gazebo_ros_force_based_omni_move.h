#ifndef GAZEBO_ROS_FORCE_BASED_OMNI_MOVE_H
#define GAZEBO_ROS_FORCE_BASED_OMNI_MOVE_H

#include <memory>
#include <thread>
#include <iostream>
#include <functional>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
//#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "pid.h"

namespace gazebo {

class GazeboRosForceBasedOmniMove : public ModelPlugin {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  protected:
    virtual void UpdateChild();
    virtual void FiniChild();

    //typedef roborts_gazebo_plugins::OmniMovePidConfig OmniMovePidConfig;
    //void DynamicReconfigureCallback(GazeboRosForceBasedOmniMove::OmniMovePidConfig &config,
    //                                uint32_t level);

  private:
    void publishOdometry(double dt);
    void queueThread();
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg);

  private:
    // xml read args
    const std::string class_name = "GazeboRosForceBasedOmniMove";
    std::string link_name_;
    std::string tf_prefix_;
    std::string robot_namespace_;
    std::string cmd_vel_topic_;
    std::string odom_topic_;
    std::string odom_frame_;
    std::string robot_base_frame_;
    double odom_rate_;
    bool publish_odom_tf_;

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    //std::shared_ptr<dynamic_reconfigure::Server<OmniMovePidConfig>> dynamic_reconfigure_server_;
    std::shared_ptr<Pid<double>> vel_x_pid_;
    std::shared_ptr<Pid<double>> vel_y_pid_;
    std::shared_ptr<Pid<double>> vel_rot_pid_;

    // Gazebo fields
    physics::ModelPtr parent_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_;

    ros::CallbackQueue queue_;
    std::thread callback_queue_thread_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;

    nav_msgs::Odometry odom_;
    tf::Transform odom_transform_;

    std::mutex lock_;
    std::mutex link_lock_;

    double vel_x_;
    double vel_y_;
    double vel_rot_;
    bool running_;
    common::Time last_odom_publish_time_;

    bool use_ground_truth_odom_;


#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d last_odom_pose_;
#else
  math::Pose last_odom_pose_;
#endif

};

}

#endif //GAZEBO_ROS_FORCE_BASED_OMNI_MOVE_H
