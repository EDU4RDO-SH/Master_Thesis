#include <control_inputs_normalizer.h>

// constructor -----------------------------------------------------------------
Normalizer::Normalizer( const ros::NodeHandle &nh,
                        const ros::NodeHandle &private_nh):
    nh_(nh),
    private_nh_(private_nh)
{

    ROS_INFO_ONCE("Normalizer object instance");

    // load parameters from parameter server
    loadParameters();

    rpyrt_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1, &Normalizer::rollPitchYawRateThrustCallback, this);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


    // initialize twist components, all set to zero
    twist_msg_.linear.x = 0.0;
    twist_msg_.linear.y = 0.0;
    twist_msg_.linear.z = 0.0;
    twist_msg_.angular.x = 0.0;
    twist_msg_.angular.y = 0.0;
    twist_msg_.angular.z = 0.0;

}


// destructor ------------------------------------------------------------------
Normalizer::~Normalizer(){

}


double Normalizer::rad2deg(double rad_angle){
    double deg_angle = rad_angle * 180.0 / M_PI;
    return deg_angle;
}

void Normalizer::loadParameters(){
    ROS_INFO("Loading scaling parameters from Parameter Server");


    if(!private_nh_.getParam("max_tilt_angle", max_tilt_angle_)){
        ROS_ERROR("max_tilt_angle is not loaded from ros parameter server");
        abort();
    }


    if(!private_nh_.getParam("max_vert_speed", max_vert_speed_)){
        ROS_ERROR("max_vert_speed is not loaded from ros parameter server");
        abort();
    }


    if(!private_nh_.getParam("max_rot_speed", max_rot_speed_)){
        ROS_ERROR("max_rot_speed is not loaded from ros parameter server");
        abort();
    }


    ROS_INFO_STREAM("max_tilt_angle: " << max_tilt_angle_ << " [deg]");
    ROS_INFO_STREAM("max_vert_speed: " << max_vert_speed_ << " [m/sec]");
    ROS_INFO_STREAM("max_rot_speed: " << max_rot_speed_ << " [deg/sec]");

}


void Normalizer::rollPitchYawRateThrustCallback(const mav_msgs::RollPitchYawrateThrust &rpyrt_msg){

    roll_ = rad2deg(-rpyrt_msg.roll) / max_tilt_angle_;
    pitch_ = rad2deg(rpyrt_msg.pitch) / max_tilt_angle_;
    vertical_vel_ = rpyrt_msg.thrust.z / max_vert_speed_;
    yaw_rate_ = rad2deg(rpyrt_msg.yaw_rate) / max_rot_speed_;

    twist_msg_.linear.x = CLAMP(pitch_, -1, 1);
    twist_msg_.linear.y = CLAMP(roll_, -1, 1);
    twist_msg_.linear.z = CLAMP(vertical_vel_, -1, 1);
    twist_msg_.angular.z = CLAMP(yaw_rate_, -1, 1);

    FILTER_SMALL_VALS(twist_msg_.linear.x, 0.01);
    FILTER_SMALL_VALS(twist_msg_.linear.y, 0.01);
    FILTER_SMALL_VALS(twist_msg_.linear.z, 0.01);
    FILTER_SMALL_VALS(twist_msg_.angular.z, 0.01);

}

void Normalizer::twist_publisher(){

    twist_pub_.publish(twist_msg_);
}


// main program ----------------------------------------------------------------

int main(int argc, char **argv){

    ros::init(argc, argv, "control_inputs_normalizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    Normalizer normalizer(nh, private_nh);  // object instance

    ros::Rate loop_rate(200);               // publishing rate

    while(ros::ok()){
        normalizer.twist_publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
