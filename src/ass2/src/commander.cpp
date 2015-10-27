#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/TwistStamped.h"
#include "dynamixel_msgs/JointState.h"

#include <sstream>

bool isrunning = true;

class Commander {
public: 
    Commander(ros::NodeHandle n_);
    void motor_geometry_callback(const geometry_msgs::TwistStamped::ConstPtr& vector);
    void servo_geometry_callback(const geometry_msgs::TwistStamped::ConstPtr& vector);
    void servo_state_callback(const dynamixel_msgs::JointState::ConstPtr& state);

private:
    ros::NodeHandle n;
    
    // Wheel movement topics
    ros::Publisher motor_1_commander;
    ros::Publisher motor_2_commander;
    ros::Publisher motor_3_commander;
    ros::Publisher motor_4_commander;

    // Arm movement topics
    ros::Publisher servo_1_commander;
    ros::Publisher servo_2_commander;
    ros::Publisher servo_3_commander;
   
    // geometry published by joy_stick_converter
    ros::Subscriber motor_geometry_sub;
    ros::Subscriber servo_geometry_sub;
    ros::Subscriber servo_1_state_sub;
    ros::Subscriber servo_2_state_sub;
    ros::Subscriber servo_3_state_sub;

    // Current positions of arm servos
    std_msgs::Float64 servo_1_position;
    std_msgs::Float64 servo_2_position;
    std_msgs::Float64 servo_3_position;

    void forward(double speed);
    void left(double speed);
    void right(double speed);
    void back(double speed);
};

Commander::Commander(ros::NodeHandle n_): n(n_) {
    motor_1_commander = n.advertise<std_msgs::Float64>("/pan_controller_1/command", 1000);
    motor_2_commander = n.advertise<std_msgs::Float64>("/pan_controller_2/command", 1000);
    motor_3_commander = n.advertise<std_msgs::Float64>("/pan_controller_3/command", 1000);
    motor_4_commander = n.advertise<std_msgs::Float64>("/pan_controller_4/command", 1000);

    servo_1_commander = n.advertise<std_msgs::Float64>("/tilt_controller_1/command", 1000);
    servo_2_commander = n.advertise<std_msgs::Float64>("/tilt_controller_2/command", 1000);
    servo_3_commander = n.advertise<std_msgs::Float64>("/tilt_controller_3/command", 1000);

    servo_1_state_sub = n.subscribe("servo_controller_1/state", 1, &Commander::servo_state_callback, this);
    servo_2_state_sub = n.subscribe("servo_controller_2/state", 1, &Commander::servo_state_callback, this);
    servo_3_state_sub = n.subscribe("servo_controller_3/state", 1, &Commander::servo_state_callback, this);

    motor_geometry_sub = n.subscribe("motor_command_vector", 1, &Commander::motor_geometry_callback, this);
    servo_geometry_sub = n.subscribe("servo_command_vector", 1, &Commander::servo_geometry_callback, this);

    servo_1_position.data = 0;
    servo_2_position.data = 0;
    servo_3_position.data = 0;
}

void Commander::motor_geometry_callback(const geometry_msgs::TwistStamped::ConstPtr& vector) {
    geometry_msgs::Twist v = vector->twist;
    left(v.linear.x - v.linear.y);
    right(v.linear.x + v.linear.y);
}

void Commander::servo_geometry_callback(const geometry_msgs::TwistStamped::ConstPtr& vector) {
    geometry_msgs::Twist v = vector->twist;

    // Increment positions
    // All positions are in radians
    servo_1_position.data += (0.05*v.linear.x);
    servo_1_position.data += (0.07*v.linear.z);

    servo_2_position.data += (0.07*v.linear.z);

    servo_3_position.data += (0.05*v.linear.y);


    // Set angular position if A or B is pushed

    if( v.angular.x != 0 ||
        v.angular.z != 0 ||
        v.angular.y != 0 ) {

        servo_1_position.data = v.angular.x;
        servo_2_position.data = v.angular.z; 
        servo_3_position.data = v.angular.y;
    }


    // Limits

    // Middle servo 
    std_msgs::Float64 servo_1_min;
    std_msgs::Float64 servo_1_max;
    servo_1_min.data = -1.4;
    servo_1_max.data = 1.4;

    // Bottom servo 
    std_msgs::Float64 servo_2_min;
    std_msgs::Float64 servo_2_max;
    servo_2_min.data = -1.3;
    servo_2_max.data = 2.8;


    // Top servo
    std_msgs::Float64 servo_3_min;
    std_msgs::Float64 servo_3_max;
    servo_3_min.data = -1.5;
    servo_3_max.data = 1.5;

    if( servo_1_position.data < servo_1_min.data) {
        servo_1_position.data = servo_1_min.data;
    }
    else if( servo_1_position.data > servo_1_max.data ) {
        servo_1_position.data = servo_1_max.data;
    }

    if( servo_2_position.data < servo_2_min.data) {
        servo_2_position.data = servo_2_min.data;
    }
    else if( servo_2_position.data > servo_2_max.data ) {
        servo_2_position.data = servo_2_max.data;
    }

    if( servo_3_position.data < servo_3_min.data) {
        servo_3_position.data = servo_3_min.data;
    }
    else if( servo_3_position.data > servo_3_max.data ) {
        servo_3_position.data = servo_3_max.data;
    }

    servo_1_commander.publish(servo_1_position);
    servo_2_commander.publish(servo_2_position);
    servo_3_commander.publish(servo_3_position);

}


void Commander::servo_state_callback(const dynamixel_msgs::JointState::ConstPtr& state) {
    if( state->name == "position_joint_1" ) {
        servo_1_position.data = state->goal_pos;       
    }
    else if( state->name == "position_joint_2" ) {
        servo_2_position.data = state->goal_pos;       
    } else if( state->name == "position_joint_3" ) {
        servo_3_position.data = state->goal_pos;       
    }
}

void Commander::forward(double speed) {
    std_msgs::Float64 command1;
    std_msgs::Float64 command2;
    std_msgs::Float64 command3;
    std_msgs::Float64 command4;

    command1.data = speed;
    command2.data = speed;
    command3.data = -speed;
    command4.data = -speed;
    
    motor_1_commander.publish(command1);
    motor_2_commander.publish(command2);
    motor_3_commander.publish(command3);
    motor_4_commander.publish(command4);
}

void Commander::left(double speed) {
    std_msgs::Float64 command1;
    std_msgs::Float64 command2;

    command1.data = speed;
    command2.data = speed;
    
    motor_1_commander.publish(command1);
    motor_2_commander.publish(command2);
}

void Commander::right(double speed) {
    std_msgs::Float64 command3;
    std_msgs::Float64 command4;

    command3.data = -speed;
    command4.data = -speed;
    
    motor_3_commander.publish(command3);
    motor_4_commander.publish(command4);
}

void Commander::back(double speed) {
    std_msgs::Float64 command1;
    std_msgs::Float64 command2;
    std_msgs::Float64 command3;
    std_msgs::Float64 command4;

    command1.data = -speed;
    command2.data = -speed;
    command3.data = speed;
    command4.data = speed;
    
    motor_1_commander.publish(command1);
    motor_2_commander.publish(command2);
    motor_3_commander.publish(command3);
    motor_4_commander.publish(command4);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "commander");

    ros::NodeHandle n;

    Commander master_chief(n);
    ROS_INFO("Commander Initialised");

    ros::spin();
    
    return 0;

}
