#include <mpc_traj_viz.h>

// contructor ------------------------------------------------------------------
TrajViz::TrajViz(ros::NodeHandle &nh): nh_(nh){

    ROS_INFO("Instance of TrajViz class instantiated");

    // subscribers
    odom_sub_ = nh_.subscribe("/odometry", 1, &TrajViz::odometryCallback, this);
    ref_sub_ = nh_.subscribe("/command/current_reference", 1, &TrajViz::referenceCallback, this);

    // path publishers
    odom_path_pub_ = nh_.advertise<nav_msgs::Path>("/odom_path", 1);
    ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/ref_path", 1);

}


// destructor ------------------------------------------------------------------
TrajViz::~TrajViz(){
    std::cout << "Instance of TrajViz terminated" << std::endl;
}


// odometry callback function --------------------------------------------------
void TrajViz::odometryCallback(const nav_msgs::Odometry &odom_msg){
    // data of Bebop's positon will be available after take off

    // populate pose stamped message
    odom_pose_stamped_msg_.header.seq = 0;
    odom_pose_stamped_msg_.header.stamp = ros::Time::now();
    odom_pose_stamped_msg_.header.frame_id = ' ';
    odom_pose_stamped_msg_.pose = odom_msg.pose.pose;

    // populate path message
    odom_path_msg_.header.seq = 0;
    odom_path_msg_.header.stamp = ros::Time::now();
    odom_path_msg_.header.frame_id = "odom";
    odom_path_msg_.poses.push_back(odom_pose_stamped_msg_);

    // publish path
    odom_path_pub_.publish(odom_path_msg_);
}


void TrajViz::referenceCallback(const trajectory_msgs::MultiDOFJointTrajectory &ref_msg){

    // populate pose stamped message
    ref_pose_stamped_msg_.header.seq = 0;
    ref_pose_stamped_msg_.header.stamp = ros::Time::now();
    ref_pose_stamped_msg_.header.frame_id = ' ';
    ref_pose_stamped_msg_.pose.position.x = ref_msg.points[0].transforms[0].translation.x;
    ref_pose_stamped_msg_.pose.position.y = ref_msg.points[0].transforms[0].translation.y;
    ref_pose_stamped_msg_.pose.position.z = ref_msg.points[0].transforms[0].translation.z;
    ref_pose_stamped_msg_.pose.orientation = ref_msg.points[0].transforms[0].rotation;

    // populate path message
    ref_path_msg_.header.seq = 0;
    ref_path_msg_.header.stamp = ros::Time::now();
    ref_path_msg_.header.frame_id = "odom";
    ref_path_msg_.poses.push_back(ref_pose_stamped_msg_);

    // publish path
    ref_path_pub_.publish(ref_path_msg_);
}




// main ************************************************************************
int main(int argc, char **argv){

    ros::init(argc, argv, "trajectory_visualizer_node");
    ros::NodeHandle nh;

    TrajViz trajViz(nh);    // object instance

    ros::spin();

    return 0;

}
