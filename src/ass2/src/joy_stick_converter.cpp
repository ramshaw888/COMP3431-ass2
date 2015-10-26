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
    ros::Publisher servo_vector;
    ros::Subscriber joyStickSubscriber;
};

JoyStickConverter::JoyStickConverter(ros::NodeHandle n_): n(n_) {
    motor_vector = n.advertise<geometry_msgs::TwistStamped>("motor_command_vector", 1000);
    servo_vector = n.advertise<geometry_msgs::TwistStamped>("servo_command_vector", 1000);
    joyStickSubscriber = n.subscribe("joy", 1, &JoyStickConverter::joyStickCallback, this);
}

void JoyStickConverter::joyStickCallback(const sensor_msgs::Joy::ConstPtr& joystick_message) {

    // Pan motors
    geometry_msgs::TwistStamped vector;
    if(joystick_message->buttons[4] == 0) {
        vector.twist.linear.x = 0;
        vector.twist.linear.y = 0;
        vector.twist.angular.z = 0;
        motor_vector.publish(vector);
    } else {
        vector.twist.linear.x = joystick_message->axes[1] * 10;
        vector.twist.linear.y = joystick_message->axes[0] * 10;
        vector.twist.angular.z = 0;
        motor_vector.publish(vector);
    }

    // Tilt motors
    if(joystick_message->buttons[5] != 0) {
        vector.twist.angular.x = joystick_message->axes[4];
        vector.twist.angular.y = joystick_message->axes[3];
        vector.twist.angular.z = joystick_message->axes[7];
        servo_vector.publish(vector);
    }

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

}
