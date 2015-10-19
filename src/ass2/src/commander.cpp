#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

bool isrunning = true;

int main(int argc, char **argv) {

    ros::init(argc, argv, "commander");

    ros::NodeHandle n;

    ros::Publisher motor_1_commander = n.advertise<std_msgs::Float64>("/pan_controller_1/command", 1000);
    ros::Publisher motor_2_commander = n.advertise<std_msgs::Float64>("/pan_controller_2/command", 1000);
    ros::Publisher motor_3_commander = n.advertise<std_msgs::Float64>("/pan_controller_3/command", 1000);
    ros::Publisher motor_4_commander = n.advertise<std_msgs::Float64>("/pan_controller_4/command", 1000);
    std_msgs::Float64 command1;
    std_msgs::Float64 command2;
    std_msgs::Float64 command3;
    std_msgs::Float64 command4;

    ros::Rate loop_rate(1);
    while (ros::ok()) {

        if(isrunning) {
            command1.data = 0.0f;
            command2.data = 0.0f;
            command3.data = 0.0f;
            command4.data = 0.0f;
            
        }
        else {
            command1.data = 1.0f;
            command2.data = 1.0f;
            command3.data = -1.0f;
            command4.data = -1.0f;
        }
        isrunning = !isrunning;


        motor_1_commander.publish(command1);
        motor_2_commander.publish(command2);
        motor_3_commander.publish(command3);
        motor_4_commander.publish(command4);
        
        ROS_INFO("hello");


        ros::spinOnce();
        loop_rate.sleep();
    }

        command1.data = 0.0f;
        motor_1_commander.publish(command1);
        motor_2_commander.publish(command1);
        motor_3_commander.publish(command1);
        motor_4_commander.publish(command1);

}
