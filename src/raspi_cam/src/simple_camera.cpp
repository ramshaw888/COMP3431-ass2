#include <ctime>
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>
#include <unistd.h>

using namespace std;

int main ( int argc, char **argv )
{
  raspicam::RaspiCam Camera; //Camera object

  //open camera
  cout<<"Opening Camera..."<<endl; //TODO: change all cout to ROS output messages
  if ( !Camera.open()) {cerr<<"Error opening camera<<endl";return -1;}

  //wait while the camera stabilizes
  usleep (3); 
  //capture
  Camera.grab();

  //allocate memory
  unsigned char *data=new unsigned char[ Camera.getImageTypeSize (raspicam::RASPICAM_FORMAT_RGB)];

  //extract image in RGB
  Camera.retrieve ( data,raspicam::RASPICAM_FORMAT_RGB);

  //save 
  std::ofstream outFile ( "raspicam_image.ppm", std::ios::binary );
  outFile<< "P6\n" << Camera.getWidth() << " " << Camera.getHeight() << " 255\n";
  outFile.write((char*) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ));

  cout<<"Image saved at raspicam_image.ppm"<<endl;

  //free resources
  delete data;
  return 0;  
}
