#include "gazebo_ros_force_based_omni_move.h"

namespace gazebo {

void GazeboRosForceBasedOmniMove::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

  parent_ = model;

  // default pid params
  vel_x_pid_ = std::make_shared<Pid<double>>(50000,-50000,20000,-20000,10000,0,0);
  vel_y_pid_ = std::make_shared<Pid<double>>(50000,-50000,20000,-20000,10000,0,0);
  vel_rot_pid_ = std::make_shared<Pid<double>>(10000,-10000,2000,-2000,400,5,0);

  // Parse the xml arguments
  robot_base_frame_ = "base_link";
  if (!sdf->HasElement("robot_base_frame"))
  {
    ROS_WARN("%s: missing element <robot_base_frame>, default to [%s]", class_name.c_str(),robot_base_frame_.c_str());
  }
  else
  {
    robot_base_frame_ = sdf->GetElement("robot_base_frame")->Get<std::string>();
  }

  // get the base as the physics link
  link_ = parent_->GetLink(robot_base_frame_);

  robot_namespace_ = "";
  if(!sdf->HasElement("robot_namespace")){
    ROS_WARN("%s: missing element <robot_namespace>, default to [%s]", class_name.c_str(),robot_namespace_.c_str());
  } else{
    robot_namespace_ = sdf->GetElement("robot_namespace")->Get<std::string>();
  }

  cmd_vel_topic_ = "cmd_vel";
  if(!sdf->HasElement("cmd_vel_topic")){
    ROS_WARN("%s: missing element <cmd_vel_topic>, default to [%s]", class_name.c_str(),cmd_vel_topic_.c_str());
  } else{
    cmd_vel_topic_ = sdf->GetElement("cmd_vel_topic")->Get<std::string>();
  }

  odom_topic_ = "odom";
  if(!sdf->HasElement("odom_topic")){
    ROS_WARN("%s: missing element <odom_topic>, default to [%s]", class_name.c_str(),odom_topic_.c_str());
  } else{
    odom_topic_ = sdf->GetElement("odom_topic")->Get<std::string>();
  }

  odom_frame_ = "odom";
  if (!sdf->HasElement("odom_frame"))
  {
    ROS_WARN("%s: missing element <odom_frame>, default to [%s]", class_name.c_str(),odom_frame_.c_str());
  }
  else
  {
    odom_frame_ = sdf->GetElement("odom_frame")->Get<std::string>();
  }

  odom_rate_ = 15.0;
  if (!sdf->HasElement("odom_rate"))
  {
    ROS_WARN("%s: missing element <odom_rate>, default to [%f]", class_name.c_str(),odom_rate_);
  }
  else
  {
    odom_rate_ = sdf->GetElement("odom_rate")->Get<double>();
  }

  publish_odom_tf_ = true;
  if (!sdf->HasElement("publish_odom_tf")) {
    ROS_WARN("%s: missing element <odom_rate>, default to true", class_name.c_str());
  } else {
    publish_odom_tf_ = sdf->GetElement("publish_odom_tf")->Get<bool>();
  }

  if (sdf->HasElement("rotation_velocity_p_gain")) {
    double rot_p_gain = sdf->GetElement("rotation_velocity_p_gain")->Get<double>();
    vel_rot_pid_->SetKp(rot_p_gain);
  }
  if (sdf->HasElement("velocity_x_p_gain")) {
    double vel_x_p_gain = sdf->GetElement("velocity_x_p_gain")->Get<double>();
    vel_x_pid_->SetKp(vel_x_p_gain);
  }
  if (sdf->HasElement("velocity_y_p_gain")) {
    double vel_y_p_gain = sdf->GetElement("velocity_y_p_gain")->Get<double>();
    vel_y_pid_->SetKp(vel_y_p_gain);
  }

  if (sdf->HasElement("use_ground_truth_odom")) {
    use_ground_truth_odom_ = sdf->GetElement("use_ground_truth_odom")->Get<bool>();
  }
  else {
    use_ground_truth_odom_ = false;
  }


#if (GAZEBO_MAJOR_VERSION >= 8)
  last_odom_publish_time_ = parent_->GetWorld()->SimTime();
  last_odom_pose_ = parent_->WorldPose();
#else
  last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
  last_odom_pose_ = parent_->GetWorldPose();
#endif

  vel_x_ = 0;
  vel_y_ = 0;
  vel_rot_ = 0;
  running_ = true;
  odom_transform_.setIdentity();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM( class_name << ": (ns = " << robot_namespace_
                                               << "). A ROS node for Gazebo has not been initialized, ");
    return;
  }
  nh_.reset(new ros::NodeHandle(robot_namespace_));

  tf_prefix_ = tf::getPrefixParam(*nh_);

  if(publish_odom_tf_){
    transform_broadcaster_.reset(new tf::TransformBroadcaster());
  }

  ros::SubscribeOptions sub_opt = ros::SubscribeOptions::create<geometry_msgs::Twist>(cmd_vel_topic_, 1,
                                                                                      std::bind(&GazeboRosForceBasedOmniMove::cmdVelCallback, this, std::placeholders::_1),
                                                                                      ros::VoidPtr(), &queue_);
  cmd_vel_sub_ = nh_->subscribe(sub_opt);
  callback_queue_thread_ = std::thread(std::bind(&GazeboRosForceBasedOmniMove::queueThread, this) );

  //if (odom_rate_ > 0.0)
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_, 1);

  //dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<OmniMovePidConfig>>(ros::NodeHandle(*nh_, "roborts_omni_move/config"));
  //dynamic_reconfigure_server_->setCallback(boost::bind(&GazeboRosForceBasedOmniMove::DynamicReconfigureCallback,this,_1,_2));

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosForceBasedOmniMove::UpdateChild, this));

}

void GazeboRosForceBasedOmniMove::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg) {
  std::lock_guard<std::mutex> lock(lock_);
  vel_x_ = cmd_vel_msg->linear.x;
  vel_y_ = cmd_vel_msg->linear.y;
  vel_rot_ = cmd_vel_msg->angular.z;
}

void GazeboRosForceBasedOmniMove::UpdateChild() {

#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time current_time = parent_->GetWorld()->SimTime();
#else
  common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
  common::Time dt = current_time - last_odom_publish_time_;

#if (GAZEBO_MAJOR_VERSION >= 8)
  //ignition::math::Pose3d pose = parent_->WorldPose();
  ignition::math::Vector3d angular_vel = parent_->WorldAngularVel();

  std::lock_guard<std::mutex> lock(link_lock_);
  link_->AddTorque(ignition::math::Vector3d(0.0, 0.0, vel_rot_pid_->Update(dt.Double(), vel_rot_, angular_vel.Z() )));

  //float yaw = pose.Rot().Yaw();

  ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

  link_->AddRelativeForce(ignition::math::Vector3d(vel_x_pid_->Update(dt.Double(), vel_x_, linear_vel.X()),
                                                   vel_y_pid_->Update(dt.Double(), vel_y_, linear_vel.Y()),
                                                   0.0) );
#else
  math::Pose pose = parent_->GetWorldPose();

  math::Vector3 angular_vel = parent_->GetWorldAngularVel();

  link_->AddTorque(math::Vector3(0.0, 0.0, vel_rot_pid_->Update(dt.Double(), vel_rot_, angular_vel.z)));

  math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

  link_->AddRelativeForce(math::Vector3(vel_x_pid_->Update(dt.Double(), vel_x_, linear_vel.x),
                                        vel_y_pid_->Update(dt.Double(), vel_y_, linear_vel.y),
                                        0.0));
#endif

  if (odom_rate_ > 0.0) {
    double seconds_since_last_update =
        (current_time - last_odom_publish_time_).Double();
    if (seconds_since_last_update > (1.0 / odom_rate_)) {
      publishOdometry(seconds_since_last_update);
      last_odom_publish_time_ = current_time;
    }
  }
}

void GazeboRosForceBasedOmniMove::FiniChild() {
  running_ = false;
  queue_.clear();
  queue_.disable();
  nh_->shutdown();
  callback_queue_thread_.join();
}

void GazeboRosForceBasedOmniMove::publishOdometry(double dt) {

  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, odom_frame_);
  std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);


#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
      ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();
      ignition::math::Pose3d   pose = parent_->WorldPose();

    if (use_ground_truth_odom_) {
        odom_transform_.setOrigin(
                tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z())
        );
        odom_transform_.setRotation(
                tf::createQuaternionFromRPY(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw())
        );
    } else {
        tf::Transform transfrom_from_motion;
        transfrom_from_motion.setIdentity();
        double x_change = linear_vel.X() * dt;
        double y_change = linear_vel.Y() * dt;
        double angle_change = angular_vel.Z() * dt;
        transfrom_from_motion.setOrigin(tf::Vector3(x_change,
                                                    y_change,
                                                    0.0));
        transfrom_from_motion.setRotation(tf::createQuaternionFromYaw(angle_change));
        odom_transform_= odom_transform_ * transfrom_from_motion;
    }

      tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
      odom_.twist.twist.angular.z = angular_vel.Z();
      odom_.twist.twist.linear.x  = linear_vel.X();
      odom_.twist.twist.linear.y  = linear_vel.Y();

#else
      math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
  math::Vector3 linear_vel = parent_->GetRelativeLinearVel();
  math::Pose pose = parent_->GetWorldPose();

  if (use_ground_truth_odom_) {
      odom_transform_.setOrigin(
                  tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z )
                  );
      odom_transform_.setRotation(
                  pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w )
                  );
  else {
      tf::Transform transfrom_from_motion;
      transfrom_from_motion.setIdentity();
      double x_change = linear_vel.x * dt;
      double y_change = linear_vel.y * dt;
      double angle_change = angular_vel.z * dt;
      transfrom_from_motion.setOrigin(tf::Vector3(x_change,
                                                  y_change,
                                                  0.0));
      transfrom_from_motion.setRotation(tf::createQuaternionFromYaw(angle_change));
      odom_transform_= odom_transform_ * transfrom_from_motion;
  }

  tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
  odom_.twist.twist.angular.z = angular_vel.z;
  odom_.twist.twist.linear.x  = linear_vel.x;
  odom_.twist.twist.linear.y  = linear_vel.y;
#endif

  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  if (transform_broadcaster_.get() ) {
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(odom_transform_, current_time, odom_frame,
                             base_footprint_frame));
  }

  if (!use_ground_truth_odom_)
  {
      odom_.pose.covariance[0] = 0.001;
      odom_.pose.covariance[7] = 0.001;
      odom_.pose.covariance[14] = 1000000000000.0;
      odom_.pose.covariance[21] = 1000000000000.0;
      odom_.pose.covariance[28] = 1000000000000.0;

#if (GAZEBO_MAJOR_VERSION >= 8)
      if (std::abs(angular_vel.Z()) < 0.0001)
#else
          if (std::abs(angular_vel.z) < 0.0001)
#endif
      {
          odom_.pose.covariance[35] = 0.01;
      } else {
          odom_.pose.covariance[35] = 100.0;
      }

      odom_.twist.covariance[0] = 0.001;
      odom_.twist.covariance[7] = 0.001;
      odom_.twist.covariance[14] = 0.001;
      odom_.twist.covariance[21] = 1000000000000.0;
      odom_.twist.covariance[28] = 1000000000000.0;

#if (GAZEBO_MAJOR_VERSION >= 8)
      if (std::abs(angular_vel.Z()) < 0.0001)
#else
          if (std::abs(angular_vel.z) < 0.0001)
#endif
      {
          odom_.twist.covariance[35] = 0.01;
      } else {
          odom_.twist.covariance[35] = 100.0;
      }
  }

  odom_pub_.publish(odom_);
}

void GazeboRosForceBasedOmniMove::queueThread() {
  static const double timeout = 0.05;
  while (running_ && nh_->ok()) {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

/*
void GazeboRosForceBasedOmniMove::DynamicReconfigureCallback(roborts_gazebo_plugins::OmniMovePidConfig &config,
                                                             uint32_t level) {
  std::lock_guard<std::mutex> lock(lock_);
  if (level == 1) {
    vel_x_pid_->SetKp(config.vel_x_kp);
    vel_y_pid_->SetKp(config.vel_y_kp);
    vel_rot_pid_->SetKp(config.vel_rot_kp);
    vel_x_pid_->SetKi(config.vel_x_ki);
    vel_y_pid_->SetKi(config.vel_y_ki);
    vel_rot_pid_->SetKi(config.vel_rot_ki);
    vel_x_pid_->SetKd(config.vel_x_kd);
    vel_y_pid_->SetKd(config.vel_y_kd);
    vel_rot_pid_->SetKd(config.vel_rot_kd);
  } else{
    config.vel_x_kp = vel_x_pid_->GetKp();
    config.vel_y_kp = vel_y_pid_->GetKp();
    config.vel_rot_kp = vel_rot_pid_->GetKp();
    config.vel_x_ki = vel_x_pid_->GetKi();
    config.vel_y_ki = vel_y_pid_->GetKi();
    config.vel_rot_ki = vel_rot_pid_->GetKi();
    config.vel_x_kd = vel_x_pid_->GetKd();
    config.vel_y_kd = vel_y_pid_->GetKd();
    config.vel_rot_kd = vel_rot_pid_->GetKd();
  }
}
*/
GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceBasedOmniMove)

}
