#ifndef MPC_TRAJ_VIZ_H_
#define MPC_TRAJ_VIZ_H_

// main ROS header
#include <ros/ros.h>

// generically libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


// class definition
class TrajViz{

private:
    ros::NodeHandle nh_;

    // subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber ref_sub_;

    // publishers
    ros::Publisher odom_path_pub_;
    ros::Publisher ref_path_pub_;


    // odometry variables
    geometry_msgs::PoseStamped odom_pose_stamped_msg_;
    nav_msgs::Path odom_path_msg_;


    // reference variables
    geometry_msgs::PoseStamped ref_pose_stamped_msg_;
    nav_msgs::Path ref_path_msg_;


public:

    // constructor/destructor
    TrajViz(ros::NodeHandle &nh);
    ~TrajViz();


    // callback functions
    void odometryCallback(const nav_msgs::Odometry &odom_msg);
    void referenceCallback(const trajectory_msgs::MultiDOFJointTrajectory &ref_msg);


};  // end of class definition


#endif
