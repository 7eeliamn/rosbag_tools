#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"

#include <cv_bridge/cv_bridge.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

using namespace cv;

// ros::Publisher cloud_pub;
int img_num=0;
int pcd_num=0;

std::string type = ".";
std::string path = "";


void cameraCallback(const sensor_msgs::Image::ConstPtr& img)
{
	
  double timeimg = img->header.stamp.toSec();

  char str[256];
  sprintf(str, "%lf", timeimg);

  std::string s_timescan = str;

  cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(img, "8UC3");

  imwrite( path + "/" + s_timescan + type, cv_cam->image );
  std::cout<< "image saved  " << img_num++ << "  complete!"<<std::endl;
}


int main(int argc, char **argv)
{
 ros::init(argc, argv, "imageSaver");

 if(argc < 3) {
    std::cout << "Please enter at least 2 parameters" << std::endl;
    std::cout << "example: rosrun pcd_image_save imageSaver [ image_type ] [ /path/to/save/image ]" << std::endl;
    return 0;
 }
 type += std::string(argv[1]);
 path = std::string(argv[2]);

 ros::NodeHandle n;
	
 ros::Subscriber sub1 = n.subscribe("/image", 2000, cameraCallback);

 ros::spin();

 return 0;
}

