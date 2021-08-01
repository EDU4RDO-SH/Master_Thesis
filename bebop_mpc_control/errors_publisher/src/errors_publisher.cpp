#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <errors_publisher/StateErrors.h>
#include <errors_publisher/Orientation.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

// global vars
trajectory_msgs::MultiDOFJointTrajectory ref_states;
nav_msgs::Odometry curr_states;
errors_publisher::StateErrors errors;


ros::Subscriber ref_sub;
ros::Subscriber odom_sub;
ros::Publisher errors_pub;


// converts quaternion to RPY angles *******************************************
errors_publisher::Orientation quaternionToRPY(const geometry_msgs::Quaternion &quaternion){
    tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    errors_publisher::Orientation orientation;
    orientation.roll = roll;
    orientation.pitch = pitch;
    orientation.yaw = yaw;

    return orientation;
}



// callback functions **********************************************************
void referenceCallback(const trajectory_msgs::MultiDOFJointTrajectory &ref_msg){
    ref_states = ref_msg;

    // reference states
    double x_ref = ref_states.points[0].transforms[0].translation.x;
    double y_ref = ref_states.points[0].transforms[0].translation.y;
    double z_ref = ref_states.points[0].transforms[0].translation.z;

    geometry_msgs::Quaternion q_ref = ref_states.points[0].transforms[0].rotation;
    errors_publisher::Orientation ref_att = quaternionToRPY(q_ref);


    // current states
    double x_curr = curr_states.pose.pose.position.x;
    double y_curr = curr_states.pose.pose.position.y;
    double z_curr = curr_states.pose.pose.position.z;

    geometry_msgs::Quaternion q_curr = curr_states.pose.pose.orientation;
    errors_publisher::Orientation curr_att = quaternionToRPY(q_curr);


    // publish errors
    errors.header.seq = 0;
    errors.header.stamp = ros::Time::now();
    errors.header.frame_id = ' ';

    errors.position.x = x_ref - x_curr;
    errors.position.y = y_ref - y_curr;
    errors.position.z = z_ref - z_curr;

    errors.orientation.roll = ref_att.roll - curr_att.roll;
    errors.orientation.pitch = ref_att.pitch - curr_att.pitch;
    errors.orientation.yaw = ref_att.yaw - curr_att.yaw;


    errors_pub.publish(errors);
}

void odometryCallback(const nav_msgs::Odometry &odom_msg){
    curr_states = odom_msg;

}



// main program ****************************************************************
int main(int argc, char **argv){

    ros::init(argc, argv, "errors_publisher_node");
    ros::NodeHandle nh;

    ref_sub = nh.subscribe("command/current_reference", 1, referenceCallback);
    odom_sub = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, odometryCallback);
    errors_pub = nh.advertise<errors_publisher::StateErrors>("state_errors", 1);

    ROS_INFO("Publishing state errors");


    ros::spin();

    return 0;
}
