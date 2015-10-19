#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

int main(int argc, char **argv) {

    ros::init(argc, argv, "commander");

    ros::NodeHandle n;

    ros::Publisher motor_1_commander = n.advertise<std_msgs::Float64>("/pan_controlller_1/command", 1000);
    ros::Publisher motor_2_commander = n.advertise<std_msgs::Float64>("/pan_controlller_2/command", 1000);
    ros::Publisher motor_3_commander = n.advertise<std_msgs::Float64>("/pan_controlller_3/command", 1000);
    ros::Publisher motor_4_commander = n.advertise<std_msgs::Float64>("/pan_controlller_4/command", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        std_msgs::Float64 command1;
        std_msgs::Float64 command2;
        std_msgs::Float64 command3;
        std_msgs::Float64 command4;

        command1.data = 1.0f;
        command2.data = 1.0f;
        command3.data = 1.0f;
        command4.data = 1.0f;


        motor_1_commander.publish(command1);
        motor_2_commander.publish(command1);
        motor_3_commander.publish(command1);
        motor_4_commander.publish(command1);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}
