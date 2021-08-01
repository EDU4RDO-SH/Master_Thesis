#include <bebop_linear_mpc/linear_mpc_node.h>

namespace mav_control{

    // constructor -------------------------------------------------------------
    LinearModelPredictiveControllerNode::LinearModelPredictiveControllerNode(
        const ros::NodeHandle &nh, const ros::NodeHandle &private_nh):
        linear_mpc_(nh, private_nh),
        dyn_config_server_(private_nh),
        nh_(nh)
    {

        // dynamic reconfiguration
        dynamic_reconfigure::Server<bebop_linear_mpc::LinearMPCConfig>::CallbackType f;
        f = boost::bind(&LinearModelPredictiveControllerNode::DynConfigCallback, this, _1, _2);
        dyn_config_server_.setCallback(f);

        // subscribers
        command_trajectory_subscriber_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1, &LinearModelPredictiveControllerNode::CommandPoseCallback, this);
        command_trajectory_array_subscriber_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, &LinearModelPredictiveControllerNode::CommandTrajectoryCallback, this);
        odometry_subscriber_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &LinearModelPredictiveControllerNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay());

        // publishers
        command_publisher_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);
        current_reference_publisher_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/current_reference", 1);
        predicted_state_publisher_ = nh_.advertise<visualization_msgs::Marker>( "predicted_state", 0);
        full_predicted_state_publisher_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>( "full_predicted_state", 1);

        nh_.param<std::string>("reference_frame", reference_frame_id_, "odom");

    }


    // destructor --------------------------------------------------------------
    LinearModelPredictiveControllerNode::~LinearModelPredictiveControllerNode()
    {

    }


    // dynamic reconfiguration -------------------------------------------------
    void LinearModelPredictiveControllerNode::DynConfigCallback(bebop_linear_mpc::LinearMPCConfig &config, uint32_t level)
    {
        Eigen::Vector3d q_position;
        Eigen::Vector3d q_velocity;
        Eigen::Vector2d q_attitude;

        Eigen::Vector3d r_command;
        Eigen::Vector3d r_delta_command;
        Eigen::VectorXd control_limits(5);

        q_position << config.q_x, config.q_y, config.q_z;
        q_velocity << config.q_vx, config.q_vy, config.q_vz;
        q_attitude << config.q_roll, config.q_pitch;

        r_command << config.r_roll, config.r_pitch, config.r_thrust;
        r_delta_command << config.r_droll, config.r_dpitch, config.r_dthrust;

        control_limits << config.roll_max, config.pitch_max, config.yaw_rate_max, config.thrust_min, config.thrust_max;

        linear_mpc_.setPositionPenality(q_position);
        linear_mpc_.setVelocityPenality(q_velocity);
        linear_mpc_.setAttitudePenality(q_attitude);
        linear_mpc_.setCommandPenality(r_command);
        linear_mpc_.setDeltaCommandPenality(r_delta_command);
        linear_mpc_.setYawGain(config.K_yaw);
        linear_mpc_.setControlLimits(control_limits);

        linear_mpc_.setAltitudeIntratorGain(config.Ki_altitude);
        linear_mpc_.setXYIntratorGain(config.Ki_xy);

        linear_mpc_.setEnableIntegrator(config.enable_integrator);
        linear_mpc_.setEnableOffsetFree(config.enable_offset_free);

        linear_mpc_.applyParameters();
    }


    // set reference point -----------------------------------------------------
    bool LinearModelPredictiveControllerNode::setReference(const mav_msgs::EigenTrajectoryPoint &reference)
    {
        linear_mpc_.setCommandTrajectoryPoint(reference);
        return true;
    }


    // set reference array -----------------------------------------------------
    bool LinearModelPredictiveControllerNode::setReferenceArray(const mav_msgs::EigenTrajectoryPointDeque &reference_array)
    {
        linear_mpc_.setCommandTrajectory(reference_array);
        return true;
    }


    // set odometry ------------------------------------------------------------
    bool LinearModelPredictiveControllerNode::setOdometry(const mav_msgs::EigenOdometry &odometry)
    {
        linear_mpc_.setOdometry(odometry);
        return true;
    }


    // compute control signals -------------------------------------------------
    bool LinearModelPredictiveControllerNode::calculateRollPitchYawrateThrustCommand(mav_msgs::EigenRollPitchYawrateThrust *attitude_thrust_command)
    {
        Eigen::Vector4d rpy_thrust;
        linear_mpc_.calculateRollPitchYawrateThrustCommand(&rpy_thrust);
        attitude_thrust_command->roll = rpy_thrust(0);
        attitude_thrust_command->pitch = rpy_thrust(1);
        attitude_thrust_command->yaw_rate = rpy_thrust(2);
        attitude_thrust_command->thrust.z() = rpy_thrust(3);
        return true;
    }


    // get reference point -----------------------------------------------------
    bool LinearModelPredictiveControllerNode::getCurrentReference(mav_msgs::EigenTrajectoryPoint *reference) const
    {
        assert(reference != nullptr);
        return linear_mpc_.getCurrentReference(reference);
    }


    // get reference array -----------------------------------------------------
    bool LinearModelPredictiveControllerNode::getCurrentReference(mav_msgs::EigenTrajectoryPointDeque *reference) const
    {
        assert(reference != nullptr);
        return linear_mpc_.getCurrentReference(reference);
    }


    // get predicted state -----------------------------------------------------
    bool LinearModelPredictiveControllerNode::getPredictedState(mav_msgs::EigenTrajectoryPointDeque *predicted_state) const
    {
        assert(predicted_state != nullptr);
        return linear_mpc_.getPredictedState(predicted_state);
    }


    // command pose callback ---------------------------------------------------
    void LinearModelPredictiveControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        mav_msgs::EigenTrajectoryPoint reference;
        mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &reference);

        setReference(reference);
    }


    // command trajectory callback ---------------------------------------------
    void LinearModelPredictiveControllerNode::CommandTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
    {
        int array_size = msg->points.size();
        if(array_size == 0)
            return;

        mav_msgs::EigenTrajectoryPointDeque references;
        mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &references);

        if(references.size() == 1){
            setReference(references.at(0));
        }
        else{
            setReferenceArray(references);
        }

    }


    // odometry callback -------------------------------------------------------
    void LinearModelPredictiveControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        ROS_INFO_ONCE("Control interface got first odometry message.");
        mav_msgs::EigenOdometry odometry;
        mav_msgs::eigenOdometryFromMsg(*msg, &odometry);

        setOdometry(odometry);
    }


    // publish computed control signals ----------------------------------------
    void LinearModelPredictiveControllerNode::PublishAttitudeCommand(const mav_msgs::EigenRollPitchYawrateThrust &command) const
    {
        mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);

        mav_msgs::EigenRollPitchYawrateThrust tmp_command = command;
        tmp_command.thrust.x() = 0;
        tmp_command.thrust.y() = 0;
        tmp_command.thrust.z() = std::max(0.0, command.thrust.z());

        msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
        mav_msgs::msgRollPitchYawrateThrustFromEigen(command, msg.get());
        command_publisher_.publish(msg);
    }


    // publish current reference -----------------------------------------------
    void LinearModelPredictiveControllerNode::PublishCurrentReference()
    {
        ros::Time time_now = ros::Time::now();
        mav_msgs::EigenTrajectoryPoint current_reference;
        getCurrentReference(&current_reference);

        tf::Quaternion q;
        tf::Vector3 p;
        tf::vectorEigenToTF(current_reference.position_W, p);
        tf::quaternionEigenToTF(current_reference.orientation_W_B, q);

        tf::Transform transform;
        transform.setOrigin(p);
        transform.setRotation(q);

        transform_broadcaster_.sendTransform(tf::StampedTransform(transform, time_now, reference_frame_id_, nh_.getNamespace() + "/current_reference"));

        if(current_reference_publisher_.getNumSubscribers() > 0){
            trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
            mav_msgs::msgMultiDofJointTrajectoryFromEigen(current_reference, msg.get());
            msg->header.stamp = time_now;
            msg->header.frame_id = reference_frame_id_;
            current_reference_publisher_.publish(msg);
        }

    }


    // publish predicted state ------------------------------------------------
    void LinearModelPredictiveControllerNode::PublishPredictedState()
    {
        if(predicted_state_publisher_.getNumSubscribers() > 0){
            mav_msgs::EigenTrajectoryPointDeque predicted_state;
            getPredictedState(&predicted_state);
            visualization_msgs::Marker marker_queue;
            marker_queue.header.frame_id = reference_frame_id_;
            marker_queue.header.stamp = ros::Time();
            marker_queue.type = visualization_msgs::Marker::LINE_STRIP;
            marker_queue.scale.x = 0.05;
            marker_queue.color.a = 1.0;
            marker_queue.color.r = 1.0;

            //    marker_heading.type = visualization_msgs::Marker::ARROW;
            for(size_t i = 0; i < predicted_state.size(); i++){
                geometry_msgs::Point p;
                p.x = predicted_state.at(i).position_W(0);
                p.y = predicted_state.at(i).position_W(1);
                p.z = predicted_state.at(i).position_W(2);
                marker_queue.points.push_back(p);
            }

            predicted_state_publisher_.publish(marker_queue);
        }

        if(full_predicted_state_publisher_.getNumSubscribers() > 0){
            mav_msgs::EigenTrajectoryPointDeque predicted_state;
            getPredictedState(&predicted_state);

            trajectory_msgs::MultiDOFJointTrajectory msg;
            msgMultiDofJointTrajectoryFromEigen(predicted_state, &msg);

            //add in timestamp information
            if(!predicted_state.empty()){
                msg.header.stamp.fromNSec(predicted_state.front().timestamp_ns - predicted_state.front().time_from_start_ns);
            }

            full_predicted_state_publisher_.publish(msg);
        }
    }


    // main control algorithm --------------------------------------------------
    void LinearModelPredictiveControllerNode::ControlAlgorithm()
    {
        mav_msgs::EigenRollPitchYawrateThrust command;
        calculateRollPitchYawrateThrustCommand(&command);
        PublishAttitudeCommand(command);
        PublishCurrentReference();
        PublishPredictedState();
    }


}  // end namespace mav_control



// main program ----------------------------------------------------------------
int main(int argc, char** argv){

    ros::init(argc, argv, "LinearModelPredictiveControllerNode");
    ros::NodeHandle nh, private_nh("~");

    // object instance
    mav_control::LinearModelPredictiveControllerNode linearModelPredictiveControllerNode(nh, private_nh);

    ros::Rate loop_rate(200);   // publishing rate

    while(ros::ok()){
        linearModelPredictiveControllerNode.ControlAlgorithm();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
