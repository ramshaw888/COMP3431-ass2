#include <ctime>
#include <iostream>
#include <raspicam/raspicam.h>
#include <unistd.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"


using namespace std;

class ImagePublisher{
private:  
  raspicam::RaspiCam pi_cam; 
  unsigned char *currImg;
  
  double cameraWidth;
  double cameraHeight;
  double cameraStep; // Full row length in bytes; 3 bytes per pixel
  int imageSize;

  int width_set;
  int height_set;
  int video_stabilisation;

  ros::NodeHandle n;
  ros::Publisher publisher;

public:
  ImagePublisher(ros::NodeHandle n_);
  void capture();
  void publish();
};

ImagePublisher::ImagePublisher(ros::NodeHandle n_): n(n_)
{
  ROS_INFO("ImagePublisher: initialising");
  //initialise publisher
  publisher = n.advertise<sensor_msgs::Image>("/camera_image", 1);
  
  width_set = 320;
  height_set = 240;
  video_stabilisation = 1;
  n.getParam("/image_publisher/image_settings/width", width_set);
  n.getParam("/image_publisher/image_settings/height", height_set);
  n.getParam("/image_publisher/image_settings/video_stabilisation", video_stabilisation);
    
  //open the camera
  ROS_DEBUG("Opening Camera..");

  pi_cam.setCaptureSize(width_set,height_set);

  pi_cam.setFormat( raspicam::RASPICAM_FORMAT_RGB );
  pi_cam.setVerticalFlip( true );
  pi_cam.setHorizontalFlip( true );
  if(video_stabilisation == 1) {
      pi_cam.setVideoStabilization(true);
  } else if(video_stabilisation == 0) {
      pi_cam.setVideoStabilization(false);
  }

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


void ImagePublisher::capture()
{
  //capture an image
  pi_cam.grab();

  //extract image in RGB
  pi_cam.retrieve(currImg, raspicam::RASPICAM_FORMAT_IGNORE);
}

// Responsible creaming a sensor_msgs/image message that includes the current image
// and publishing it to 'camera/image'
void ImagePublisher::publish()
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

  ImagePublisher p(n);

  while(ros::ok())
  {
    p.capture();
    p.publish();
  
    ros::spinOnce();
    r.sleep();
  }
}
