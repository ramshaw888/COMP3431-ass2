#include "ros/ros.h"
#include "string"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include "dynamixel_msgs/JointState.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

class EmuStatePublisher {
public:
    EmuStatePublisher(ros::NodeHandle n_);
    void servo_state_callback(const dynamixel_msgs::JointState::ConstPtr& state);
private:
    ros::NodeHandle n;
    ros::Publisher joint_state_publisher;;
    ros::Subscriber servo_1_state_sub;
    ros::Subscriber servo_2_state_sub;
    ros::Subscriber servo_3_state_sub;

    std::string joint_name_1;
    std::string joint_name_2;
    std::string joint_name_3;

    std_msgs::Float64 servo_1_position;
    std_msgs::Float64 servo_2_position;
    std_msgs::Float64 servo_3_position;
};

EmuStatePublisher::EmuStatePublisher(ros::NodeHandle n_): n(n_) {
    joint_name_1 = "base_to_arm";
    joint_name_2 = "arm_to_head";
    joint_name_3 = "head_to_camera";
    servo_1_position.data = 0;
    servo_2_position.data = 0;
    servo_3_position.data = 0;

    servo_1_state_sub = n.subscribe("tilt_controller_1/state", 1, &EmuStatePublisher::servo_state_callback, this);
    servo_2_state_sub = n.subscribe("tilt_controller_2/state", 1, &EmuStatePublisher::servo_state_callback, this);
    servo_3_state_sub = n.subscribe("tilt_controller_3/state", 1, &EmuStatePublisher::servo_state_callback, this);
    joint_state_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);
}


void EmuStatePublisher::servo_state_callback(const dynamixel_msgs::JointState::ConstPtr& state) {

    sensor_msgs::JointState nextState;
    nextState.header = state->header;
    nextState.name.resize(3);
    nextState.position.resize(3);

    if( state->name == "position_joint_1" ) {
        servo_1_position.data = state->current_pos;
    }
    else if( state->name == "position_joint_2" ) {
        servo_2_position.data = state->current_pos;
    }
    else if( state->name == "position_joint_3" ) {
        servo_3_position.data = state->current_pos;
    }

    nextState.name[0] = joint_name_1;
    nextState.name[1] = joint_name_2;
    nextState.name[2] = joint_name_3;
    nextState.position[0] = -servo_2_position.data;
    nextState.position[1] = servo_1_position.data;
    nextState.position[2] = servo_3_position.data;

    joint_state_publisher.publish(nextState);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "emuStatePublisher");

    ros::NodeHandle n;

    EmuStatePublisher cortana(n);
    ROS_INFO("EmuStatePublisher Initialised");

    ros::spin();

}
