#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"

#include <sstream>

bool isrunning = true;

class Commander {
public: 
    Commander(ros::NodeHandle n_);
    void geometryCallback(const geometry_msgs::TwistStamped::ConstPtr& vector);

private:
    ros::NodeHandle n;

    ros::Publisher motor_1_commander;
    ros::Publisher motor_2_commander;
    ros::Publisher motor_3_commander;
    ros::Publisher motor_4_commander;
    ros::Publisher servo_1_commander;
    ros::Publisher servo_2_commander;
    ros::Publisher servo_3_commander;
    
    ros::Subscriber geometry_sub;

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

    geometry_sub = n.subscribe("command_vector", 1, &Commander::geometryCallback, this);
}

void Commander::geometryCallback(const geometry_msgs::TwistStamped::ConstPtr& vector) {
    ROS_INFO("COMMANDER: MESSAGE RECEIVED");
    geometry_msgs::Twist v = vector->twist;    
    if(v.angular.z == 0) {
        forward(v.linear.x);
    }
    else {
        left(v.angular.z);
    }

    std_msgs::Float64 tilt1;
    tilt1.data = -1.25;
    servo_1_commander.publish(tilt1);
    std_msgs::Float64 tilt2;
    tilt2.data = -1.75;
    servo_2_commander.publish(tilt2);
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
    std_msgs::Float64 command3;
    std_msgs::Float64 command4;

    command1.data = -speed;
    command2.data = -speed;
    command3.data = -speed;
    command4.data = -speed;
    
    motor_1_commander.publish(command1);
    motor_2_commander.publish(command2);
    motor_3_commander.publish(command3);
    motor_4_commander.publish(command4);
}

void Commander::right(double speed) {
    std_msgs::Float64 command1;
    std_msgs::Float64 command2;
    std_msgs::Float64 command3;
    std_msgs::Float64 command4;

    command1.data = speed;
    command2.data = speed;
    command3.data = speed;
    command4.data = speed;
    
    motor_1_commander.publish(command1);
    motor_2_commander.publish(command2);
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
