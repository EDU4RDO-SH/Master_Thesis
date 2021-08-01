#ifndef CONTROL_INPUTS_NORMALIZER_H_
#define CONTROL_INPUTS_NORMALIZER_H_

// main ROS header
#include <ros/ros.h>

// generically libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/default_topics.h>


// saturate values between 'h' and 'l'
// setpoint_cmd_vel.linear.x = CLAMP(setpoint_cmd_vel.linear.x, -param_max_linear_vel_, param_max_linear_vel_);
#ifndef CLAMP
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))
#endif

// set values lower than 'eps' to 0.0
// FILTER_SMALL_VALS(ctrl_twist_.linear.x, 0.01);
#ifndef FILTER_SMALL_VALS
#define FILTER_SMALL_VALS(x, eps) (x = ((fabs((x)) < (eps)) ? 0.0 : (x)))
#endif


// class definition
class Normalizer{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber rpyrt_sub_;
    ros::Publisher twist_pub_;

    // member variables
    geometry_msgs::Twist twist_msg_;


    // scaling parameters
    double max_tilt_angle_;
    double max_vert_speed_;
    double max_rot_speed_;

    // control inputs
    double roll_;
    double pitch_;
    double vertical_vel_;
    double yaw_rate_;


public:

    Normalizer(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
    ~Normalizer();

    // member methods
    double rad2deg(double rad_angle);
    void twist_publisher();
    void loadParameters();

    // callback functions
    void rollPitchYawRateThrustCallback(const mav_msgs::RollPitchYawrateThrust &rpyrt_msg);

};  // end of class definition


#endif
