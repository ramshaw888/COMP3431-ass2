#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"

class JoyStickConverter {
public:
    JoyStickConverter(ros::NodeHandle n_);
    void joyStickCallback(const sensor_msgs::Joy::ConstPtr& joystick_message); 
    void publish(geometry_msgs::TwistStamped vector);
private:
    ros::NodeHandle n;

    ros::Publisher motor_vector;

    ros::Subscriber joyStickSubscriber;
};

JoyStickConverter::JoyStickConverter(ros::NodeHandle n_): n(n_) {
    motor_vector = n.advertise<geometry_msgs::TwistStamped>("command_vector", 1000);

    joyStickSubscriber = n.subscribe("joy", 1, &JoyStickConverter::joyStickCallback, this);
    
}

void JoyStickConverter::joyStickCallback(const sensor_msgs::Joy::ConstPtr& joystick_message) {
    ROS_INFO("joystick recieved");
    geometry_msgs::TwistStamped vector;
    vector.twist.linear.x = joystick_message->axes[1] * 10;
    vector.twist.linear.y = joystick_message->axes[0] * 10;
    vector.twist.angular.z = 0;
    motor_vector.publish(vector);
}

void JoyStickConverter::publish(geometry_msgs::TwistStamped vector) {
    motor_vector.publish(vector);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "joyStickConverter");

    ros::NodeHandle n;

    JoyStickConverter cortana(n);
    ROS_INFO("JoyStickConverter Initialised");

    ros::spin();
    /*ros::Rate loop_rate(1);
    bool forward = true;
    while(ros::ok()) {
        
        geometry_msgs::TwistStamped mockMessage;
        if(forward) {
            mockMessage.twist.linear.x = 10;
            mockMessage.twist.linear.y = 0;
            mockMessage.twist.linear.z = 0;
            mockMessage.twist.angular.z = 0;
        }
        else {
            mockMessage.twist.angular.z = 10;
        }
        forward = !forward;
        cortana.publish(mockMessage);
        ros::spinOnce();
        loop_rate.sleep();
    }*/

}
