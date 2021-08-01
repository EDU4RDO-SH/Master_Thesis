/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <thread>
#include <chrono>

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


// class definition
class WaypointWithTime{

    public:
    WaypointWithTime():
        waiting_time(0),
        position(0, 0, 0),
        yaw(0){
    }

    WaypointWithTime(double t, float x, float y, float z, float _yaw):
        waiting_time(t),
        position(x, y, z),
        yaw(_yaw)  {
    }

    double waiting_time;
    Eigen::Vector3d position;
    double yaw;

};


// main program
int main(int argc, char** argv){

    ros::init(argc, argv, "bebop_waypoint_publisher");
    ros::NodeHandle nh;

    ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    ROS_INFO("Started waypoint publisher.");

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    if (args.size() != 2 && args.size() != 3) {
        ROS_ERROR("Usage: waypoint_example <waypoint_file>"
        "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
        return -1;
    }

    std::vector<WaypointWithTime> waypoints;
    const float DEG_2_RAD = M_PI / 180.0;
    const float RAD_2_DEG = 180.0 / M_PI;

    std::ifstream wp_file(args.at(1).c_str());

    if(wp_file.is_open()){
        double t, x, y, z, yaw;
        // Only read complete waypoints.
        while (wp_file >> t >> x >> y >> z >> yaw) {
            waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
        }
        wp_file.close();
        ROS_INFO("Read %d waypoints.", (int )waypoints.size());
    }
    else{
        ROS_ERROR_STREAM("Unable to open poses file: " << args.at(1));
    return -1;
    }


    // Wait for t seconds to let the Gazebo GUI show up.
    double t = 0.5;
    ros::Duration(t).sleep();

    ROS_INFO("Start publishing waypoints.");

    for(size_t i = 0; i < waypoints.size(); ++i){

        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        Eigen::Vector3d desired_position(waypoints[i].position.x(), waypoints[i].position.y(), waypoints[i].position.z());
        double desired_yaw = waypoints[i].yaw;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

        ROS_INFO("Publishing waypoint: [%f, %f, %f, %f].",
                 desired_position.x(),
                 desired_position.y(),
                 desired_position.z(),
                 desired_yaw * RAD_2_DEG);

        trajectory_pub.publish(trajectory_msg);

        // Wait for t seconds to let the Gazebo GUI show up.
        double t = waypoints[i].waiting_time;
        ros::Duration(t).sleep();

    }

    ros::spin();
}
