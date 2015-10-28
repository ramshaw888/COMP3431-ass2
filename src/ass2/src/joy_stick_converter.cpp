#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

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

    std_msgs::Float64 a_pos_x;
    std_msgs::Float64 a_pos_y;
    std_msgs::Float64 a_pos_z;

    std_msgs::Float64 b_pos_x;
    std_msgs::Float64 b_pos_y;
    std_msgs::Float64 b_pos_z;

    std_msgs::Float64 y_pos_x;
    std_msgs::Float64 y_pos_y;
    std_msgs::Float64 y_pos_z;

};

JoyStickConverter::JoyStickConverter(ros::NodeHandle n_): n(n_) {
    motor_vector = n.advertise<geometry_msgs::TwistStamped>("motor_command_vector", 1000);
    servo_vector = n.advertise<geometry_msgs::TwistStamped>("servo_command_vector", 1000);
    joyStickSubscriber = n.subscribe("joy", 1, &JoyStickConverter::joyStickCallback, this);

    a_pos_x.data = 0.0;
    a_pos_y.data = -0.78;
    a_pos_z.data = 0.0;
    
    n.getParam("/joyStickConverter/default_positions/a_pos_head", a_pos_x.data);
    n.getParam("/joyStickConverter/default_positions/a_pos_cam", a_pos_y.data);
    n.getParam("/joyStickConverter/default_positions/a_pos_base", a_pos_z.data);
    a_pos_x.data = -a_pos_x.data;
    a_pos_y.data = -a_pos_y.data;
    b_pos_x.data = -0.78;
    b_pos_y.data = -0.4;
    b_pos_z.data = 0.0;

    n.getParam("/joyStickConverter/default_positions/b_pos_head", b_pos_x.data);
    n.getParam("/joyStickConverter/default_positions/b_pos_cam", b_pos_y.data);
    n.getParam("/joyStickConverter/default_positions/b_pos_base", b_pos_z.data);
    b_pos_x.data = -b_pos_x.data;
    b_pos_y.data = -b_pos_y.data;

    y_pos_x.data = -0.78;
    y_pos_y.data = -0.4;
    y_pos_z.data = 0.0;

    n.getParam("/joyStickConverter/default_positions/y_pos_head", y_pos_x.data);
    n.getParam("/joyStickConverter/default_positions/y_pos_cam", y_pos_y.data);
    n.getParam("/joyStickConverter/default_positions/y_pos_base", y_pos_z.data);
    y_pos_x.data = -y_pos_x.data;
    y_pos_y.data = -y_pos_y.data;


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
        vector.twist.linear.x = joystick_message->axes[1] * 30;
        vector.twist.linear.y = joystick_message->axes[0] * 30;
        vector.twist.angular.z = 0;
        motor_vector.publish(vector);
    }

    // x -> Motor 1 (middle)
    // y -> Motor 3 (top camera motor)
    // z -> Motor 2 (bottom)

    // We set twist.linear values when incrementing the motors,
    // and twist.angular when setting them to a specified radian value.

    // Tilt motors
    if(joystick_message->buttons[5] != 0) {
        vector.twist.linear.x = joystick_message->axes[4];
        vector.twist.linear.y = joystick_message->axes[3];
        vector.twist.linear.z = joystick_message->axes[7];
        servo_vector.publish(vector);
    }

    // Reset to low balanced position
    // A Button
    if(joystick_message->buttons[0] == 1) {
        ROS_INFO("JoyStickConverter : A Resetting back to balanced position");
        vector.twist.angular.x = a_pos_x.data;
        vector.twist.angular.z = a_pos_z.data;
        vector.twist.angular.y = a_pos_y.data;
        servo_vector.publish(vector);
    }

    // Reset to low balanced position
    // B Button
    if(joystick_message->buttons[1] == 1) {
        ROS_INFO("JoyStickConverter : B Resetting to front view position");
        vector.twist.angular.x = b_pos_x.data;
        vector.twist.angular.z = b_pos_z.data;
        vector.twist.angular.y = b_pos_y.data;
        servo_vector.publish(vector);
    }

    // B Button
    if(joystick_message->buttons[3] == 1) {
        ROS_INFO("JoyStickConverter : Y Resetting to front view position");
        vector.twist.angular.x = y_pos_x.data;
        vector.twist.angular.z = y_pos_z.data;
        vector.twist.angular.y = y_pos_y.data;
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
