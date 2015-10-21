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
  //initialise publisher
  publisher = n.advertise<sensor_msgs::Image>("/camera/image", 1);

  //open the camera
  ROS_DEBUG("Opening Camera..");
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
  // CURRENTLY JUST SAVES TO FILE
  //std::ofstream outFile ( "raspicam_image.ppm", std::ios::binary );
  //outFile<< "P6\n" << pi_cam.getWidth() << " " << pi_cam.getHeight() << " 255\n";
  //outFile.write((char*) currImg, imageSize);

  // create image message
  sensor_msgs::Image message;
  message.header = std_msgs::Header();
  message.height = cameraHeight;
  message.width = cameraWidth;
  message.encoding = "bgr8";
  message.is_bigendian = false;
  message.step = cameraStep;
  message.data = * currImg;

  publisher.publish(message);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "imagePub");
  ros::NodeHandle n;
  ros::Rate r(100);

  ImagePub p = ImagePub();

  while(ros::ok())
  {
    p.capture();
    p.publish();
  
    ros::spinOnce();
    r.sleep();
  }
}
