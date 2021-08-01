#ifndef LINEAR_MPC_NODE_H
#define LINEAR_MPC_NODE_H

// ROS related libraries
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

// dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <bebop_linear_mpc/LinearMPCConfig.h>

// c++ libraries
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

// MPC controller library
#include <bebop_linear_mpc/linear_mpc.h>



namespace mav_control {

    class LinearModelPredictiveControllerNode{

        public:

            LinearModelPredictiveControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);  // constructor
            ~LinearModelPredictiveControllerNode(); // destructor

            void ControlAlgorithm();

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        private:

            LinearModelPredictiveController linear_mpc_;    // controller object instance

            // dynamic reconfiguration
            dynamic_reconfigure::Server<bebop_linear_mpc::LinearMPCConfig> dyn_config_server_;
            void DynConfigCallback(bebop_linear_mpc::LinearMPCConfig &config, uint32_t level);

            // member variables
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            ros::Subscriber odometry_subscriber_;
            ros::Subscriber command_trajectory_subscriber_;
            ros::Subscriber command_trajectory_array_subscriber_;

            ros::Publisher command_publisher_;
            ros::Publisher current_reference_publisher_;
            ros::Publisher predicted_state_publisher_;
            ros::Publisher full_predicted_state_publisher_;

            std::string reference_frame_id_;
            tf::TransformBroadcaster transform_broadcaster_;

            mav_msgs::EigenOdometry current_state_;
            mav_msgs::EigenTrajectoryPointDeque current_reference_queue_;

            // setters
            virtual bool setReference(const mav_msgs::EigenTrajectoryPoint& reference);
            virtual bool setReferenceArray(const mav_msgs::EigenTrajectoryPointDeque& reference_array);
            virtual bool setOdometry(const mav_msgs::EigenOdometry& odometry);
            virtual bool calculateRollPitchYawrateThrustCommand(mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command);

            // getters
            virtual bool getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;
            virtual bool getCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const;
            virtual bool getPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const;

            // member methods
            void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
            void CommandTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
            void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);

            void PublishAttitudeCommand(const mav_msgs::EigenRollPitchYawrateThrust& command) const;
            void PublishCurrentReference();
            void PublishPredictedState();

    };

}   // end of mav_control namespace

#endif
