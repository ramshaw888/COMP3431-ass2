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

class ImagePub{
private:  
  raspicam::RaspiCam pi_cam; 
  unsigned char *currImg;
  
  double cameraWidth;
  double cameraHeight;
  double cameraStep; // Full row length in bytes; 3 bytes per pixel
  int imageSize;

  ros::NodeHandle n;
  ros::Publisher publisher;

public:
  ImagePub();  
  ~ImagePub();
  void capture();
  void publish();
};

ImagePub::ImagePub()
{
  ROS_INFO("EMUUUUU");
  //initialise publisher
  publisher = n.advertise<sensor_msgs::Image>("/camera_image", 1);

  //open the camera
  ROS_DEBUG("Opening Camera..");

  pi_cam.setCaptureSize(320,240);

  pi_cam.setFormat( raspicam::RASPICAM_FORMAT_RGB );
  pi_cam.setVerticalFlip( true );
  pi_cam.setHorizontalFlip( true );
  pi_cam.setVideoStabilization( false );

  if ( !pi_cam.open()) { ROS_ERROR("Error opening camera"); return; }
  cameraWidth = pi_cam.getWidth();
  cameraHeight = pi_cam.getHeight();
  cameraStep = 3 * cameraWidth;
  imageSize = pi_cam.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB);

  //initialize memory for the current image
  currImg = new unsigned char[imageSize]; //NOTE: this image will be of type RGB8
   //wait while the camera stabilizes
  usleep(3);
}

ImagePub::~ImagePub()
{
  delete currImg;
}

void ImagePub::capture()
{
  //capture an image
  pi_cam.grab();

  //extract image in RGB
  pi_cam.retrieve(currImg, raspicam::RASPICAM_FORMAT_IGNORE);
}

// Responsible creaming a sensor_msgs/image message that includes the current image
// and publishing it to 'camera/image'
void ImagePub::publish()
{

  sensor_msgs::Image message;

  //set message variables
  message.height = cameraHeight;
  message.width = cameraWidth;
  message.encoding = sensor_msgs::image_encodings::RGB8;
  message.step = 3 * cameraWidth;
  message.data = std::vector<unsigned char>(currImg, currImg + (int)((int)cameraWidth * (int)cameraHeight) * 3);

  //publish     
  publisher.publish(message);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "imagePub");
  ros::NodeHandle n;
  ros::Rate r(1000);

  ImagePub p = ImagePub();

  while(ros::ok())
  {
    p.capture();
    p.publish();
  
    ros::spinOnce();
    r.sleep();
  }
}
