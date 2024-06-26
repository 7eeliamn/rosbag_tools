#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <string>
#include <sstream>
#include <iostream>

// 函数：创建目录并设置权限
bool create_directory(const std::string& path) {
    if (mkdir(path.c_str(), 0777) == -1) {
        if (errno != EEXIST) {
            std::cerr << "Error creating directory " << path << ": " << strerror(errno) << std::endl;
            return false;
        }
    } else {
        chmod(path.c_str(), 0777);
    }
    return true;
}

// 提取 bag 文件名字并去掉 .bag 扩展名
std::string extract_bag_name(const std::string& bag_file_path) {
    size_t last_slash_idx = bag_file_path.find_last_of("/");
    std::string filename = (last_slash_idx == std::string::npos) ? bag_file_path : bag_file_path.substr(last_slash_idx + 1);

    size_t period_idx = filename.rfind(".bag");
    if (period_idx != std::string::npos) {
        filename = filename.substr(0, period_idx);
    }
    return filename;
}

int main(int argc, char** argv) {
    if (argc < 5) {
        ROS_ERROR("Usage: rosrun your_package your_node <bag_file> <output_dir> <pc_topic> <img_topic>");
        return -1;
    }

    std::string bag_file = argv[1];
    std::string output_dir = argv[2];
    // std::string pcd_output_dir = argv[2];
    // std::string jpg_output_dir = argv[3];
    std::string pc_topic = argv[3];
    std::string img_topic = argv[4];
    int numb_img = -1;
    int numb_pcd = -1;
    if(argc == 6){
        std::string num_str = argv[5];
        numb_img = std::stoi(num_str);
        numb_pcd = numb_img;
    }

    std::string bag_name = extract_bag_name(bag_file);
    std::cout << "Bag file name: " << bag_name << ".bag" << std::endl;
    output_dir+=("/"+bag_name);
    if (!create_directory(output_dir)) {
        return -1;
    }
    
    std::string pcd_output_dir = (output_dir+"/pcd");
    std::string jpg_output_dir = (output_dir+"/img");
    // 创建输出目录
    if (!create_directory(pcd_output_dir) || !create_directory(jpg_output_dir)) {
        return -1;
    }

    ros::init(argc, argv, "bag_reader");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(pc_topic);
    topics.push_back(img_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance& m : view) {
        
        if (m.getTopic() == pc_topic || ("/" + m.getTopic() == pc_topic)) {
            sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc_msg != nullptr) {
                pcl::PointCloud<pcl::PointXYZI> cloud;
                pcl::fromROSMsg(*pc_msg, cloud);

                std::stringstream ss;
                ss << pcd_output_dir << "/" << pc_msg->header.stamp.toNSec() << ".pcd";
                pcl::io::savePCDFileBinary(ss.str(), cloud);

                // 设置文件权限为777
                chmod(ss.str().c_str(), 0777);
            }
        }
        


        if (m.getTopic() == img_topic || ("/" + m.getTopic() == img_topic)) {
            sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            if (img_msg != nullptr) {
                cv_bridge::CvImagePtr cv_ptr;
                try {
                    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    continue;
                }

                std::stringstream ss;
                ss << jpg_output_dir << "/" << img_msg->header.stamp.toNSec() << ".jpg";
                cv::imwrite(ss.str(), cv_ptr->image);

                // 设置文件权限为777
                chmod(ss.str().c_str(), 0777);
            }

        }
    }

    bag.close();
    return 0;
}
