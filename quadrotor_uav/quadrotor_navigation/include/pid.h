
#ifndef ROBORTS_GAZEBO_PLUGINS_PID_H
#define ROBORTS_GAZEBO_PLUGINS_PID_H

template<typename T>
class Pid {
 public:
  Pid(T dt,
      T ctrl_max,
      T ctrl_min,
      T integral_min,
      T integral_max,
      T kp,
      T ki,
      T kd)
      :dt_(dt),
       ctrl_max_(ctrl_max),
       ctrl_min_(ctrl_min),
       integral_max_(integral_max),
       integral_min_(integral_min),
       kp_(kp),
       ki_(ki),
       kd_(kd),
       error_(0),
       error_last_time_(0),
       integral_(0) {};

  Pid(T ctrl_max,
          T ctrl_min,
          T integral_min,
          T integral_max,
          T kp,
          T ki,
          T kd)
      :ctrl_max_(ctrl_max),
       ctrl_min_(ctrl_min),
       integral_max_(integral_max),
       integral_min_(integral_min),
       kp_(kp),
       ki_(ki),
       kd_(kd),
       error_(0),
       error_last_time_(0),
       integral_(0) {};

  virtual T Update(T target_point, T current_point) {
    return Update(dt_, target_point, current_point);
  };

  virtual T Update(T dt, T target_point, T current_point){

    error_ = target_point - current_point;
    T p_out = kp_*error_;

    integral_ += error_*dt;
    if(integral_ > integral_max_ ){
        integral_ = integral_max_;
    } else if(integral_ < integral_min_){
        integral_ = integral_min_;
    }

    T i_out = ki_*integral_;

    T derivative = (error_ - error_last_time_) / dt;
    T d_out = kd_ * derivative;

    T output = p_out + i_out + d_out;

    error_last_time_ = error_;

    if(output > ctrl_max_ ){
      return ctrl_max_;
    } else if(output < ctrl_min_){
      return ctrl_min_;
    } else{
      return output;
    }


  };

  T GetKp() const {
    return kp_;
  }
  void SetKp(T kp) {
    Pid::kp_ = kp;
  }
  T GetKi() const {
    return ki_;
  }
  void SetKi(T ki) {
    Pid::ki_ = ki;
  }
  T GetKd() const {
    return kd_;
  }
  void SetKd(T kd) {
    Pid::kd_ = kd;
  }
 private:
  T dt_;
  T kp_;
  T ki_;
  T kd_;
  T ctrl_max_;
  T ctrl_min_;
  T integral_min_;
  T integral_max_;
  T error_;
  T error_last_time_;
  T integral_;
};

#endif //ROBORTS_GAZEBO_PLUGINS_PID_H
