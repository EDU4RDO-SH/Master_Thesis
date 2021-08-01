#include <ros/ros.h>
#include <math.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>


bebop_msgs::Ardrone3PilotingStateAttitudeChanged attitude;


// callback function -----------------------------------------------------------
void attitudeCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged &att_msg){

    double roll_deg = att_msg.roll * 180.0 / M_PI;

    double pitch_deg = att_msg.pitch * 180.0 / M_PI;

    double yaw_deg = att_msg.yaw * 180.0 / M_PI;


    attitude.header = att_msg.header;
    attitude.roll = -roll_deg;
    attitude.pitch = -pitch_deg;
    attitude.yaw = -yaw_deg;
}



int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_attitude_publisher");
    ros::NodeHandle nh;

    ros::Subscriber att_sub = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, attitudeCallback);
    ros::Publisher att_pub = nh.advertise<bebop_msgs::Ardrone3PilotingStateAttitudeChanged>("bebop/attitude", 1);

    ROS_INFO("republishing bebop attitude messages");


    ros::Rate loop_rate(100);

    while(ros::ok()){
        att_pub.publish(attitude);
        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;


}
