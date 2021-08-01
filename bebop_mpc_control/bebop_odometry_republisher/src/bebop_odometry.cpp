#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>

nav_msgs::Odometry odom;

// callback function -----------------------------------------------------------
void odometryCallback(const nav_msgs::Odometry &odom_msg){

    odom.header.seq = 0;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = ' ';
    odom.pose.pose = odom_msg.pose.pose;
    odom.twist.twist = odom_msg.twist.twist;
}


// main program ----------------------------------------------------------------
int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_odometry_republisher");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odometryCallback, ros::TransportHints().tcpNoDelay());
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("bebop/odometry", 1);

    ROS_INFO("republishing bebop odometry messages");


    ros::Rate loop_rate(100);

    while(ros::ok()){
        odom_pub.publish(odom);
        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
