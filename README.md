# rosbag_tools

这个是一个关于rosbag的工具，提供rosbag解析功能，图像和点云抓取功能，支持ros和ros2.

在ros中，是一个功能包，需要您事先进行编译才能使用，功能包内有三个工具，分别如下：

第一个工具：bagReader。bagReader可以读取一个bag文件并提取指定image_topic和pointcloud_topic，最后输出到指定文件夹。
使用方式如下：
```bash
cd ros/rosbagExtracter
catkin_make
source devel/setup.sh
rosrun bagextracter bagReader < bag_file > < output_dir > < pc_topic > < img_topic >
```

第二个工具是pcdSaver。pcdSaver可以单独针对ros环境中的某一个pointcloud_topic，进行pcd文件的保存。pcdSaver是一个持续保存的工具，这意味着，只要ros环境中存在指定的pointcloud_topic，就能一直保存。
只接收topic名称为：*/points_raw*
使用方式如下：
```bash
cd ros/rosbagExtracter
catkin_make
source devel/setup.sh
rosrun bagextracter pcdSaver < /path/to/save/pcd >
```

第三个工具是imageSaver。imageSaver与pcdSaver的功能类似，针对的是ros环境中的image_topic。
只接收topic名称为 */image*
使用方式如下：
```bash
cd ros/rosbagExtracter
catkin_make
source devel/setup.sh
rosrun bagextracter imageSaver  < image_type >  < /path/to/save/image >
```

在ros2中，是一个python文件，需要您机器的环境中安装有python3的环境。
第一个工具：capture_pcd_png。capture_pcd_png可以针对ros2环境中的image_topic和pointcloud_topic，每次单独保存一张pcd文件和png文件，只要程序启动后，按下回车键保存。
使用方式如下:
```bash
cd ros2/captureOnce
./capture_pcd_png.py
```
