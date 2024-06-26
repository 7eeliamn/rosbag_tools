#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"

#include <cv_bridge/cv_bridge.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
int pcd_num=0;

std::string path = "" ;

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& scan)
{
	
  double timescan = scan->header.stamp.toSec();
	
  char str[256];
  sprintf(str, "%lf", timescan);

  std::string s_timescan = str;
	
  //std::string pcd_name(s_timescan);
	
  pcl::PointCloud<pcl::PointXYZI> cloud;  
  pcl::fromROSMsg(*scan, cloud);
  pcl::io::savePCDFileASCII (path + "/" + s_timescan + ".pcd", cloud);
  std::cout<< "pcd saved  "<< pcd_num++ <<  "  complete! " <<std::endl;
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "pcdSaver");

 if(argc < 2) {
    std::cout << "Please enter at least 1 parameters" << std::endl;
    std::cout << "example: rosrun pcd_image_save pcdSaver [ /path/to/save/pcd ]" << std::endl;
    return 0;
 }
 path = std::string(argv[1]);

 ros::NodeHandle n;
	//pcd
 ros::Subscriber velodyne = n.subscribe("/points_raw", 2000, lidarCallback);

 ros::spin();

 return 0;
}

