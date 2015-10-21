#include <ctime>
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>
#include <unistd.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"


using namespace std;

void imageCallback(const sensor_msgs::Image::ConstPtr &image) {
    ROS_INFO("Image recieved.");
    
    std::ofstream outFile ( "imageSub.ppm", std::ios::binary );
    outFile<< "P6\n" << pi_cam.getWidth() << " " << pi_cam.getHeight() << " 255\n";
    outFile.write((char*) image.data, imageSize);
}

int main (int argc, char **argv)
{
    ros:init(argc, argv, "image_listener");
    ros:NodeHandle nh;

    ros::Subscriber color_sub = n.subscribe("camera/image", 1, imageCallback);
    ros::spin();
}
