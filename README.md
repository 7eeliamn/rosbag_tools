# rosbag_tools

这个是一个关于rosbag的工具，提供rosbag解析功能，图像和点云抓取功能，支持ros和ros2.

在ros中，是一个功能包，需要您事先进行编译才能使用，功能包内有三个工具，分别如下：

## 第一个工具：bagReader。bagReader可以读取一个bag文件并提取指定image_topic和pointcloud_topic，最后输出到指定文件夹。
使用方式如下：
```bash
cd ros/rosbagExtracter
catkin_make
source devel/setup.sh
rosrun bagextracter bagReader < bag_file > < output_dir > < pc_topic > < img_topic >
```

## 第二个工具是pcdSaver。pcdSaver可以单独针对ros环境中的某一个pointcloud_topic，进行pcd文件的保存。pcdSaver是一个持续保存的工具，这意味着，只要ros环境中存在指定的pointcloud_topic，就能一直保存。
只接收topic名称为：*/points_raw*
使用方式如下：
```bash
cd ros/rosbagExtracter
catkin_make
source devel/setup.sh
rosrun bagextracter pcdSaver < /path/to/save/pcd >
```

## 第三个工具是imageSaver。imageSaver与pcdSaver的功能类似，针对的是ros环境中的image_topic。
只接收topic名称为 */image*
使用方式如下：
```bash
cd ros/rosbagExtracter
catkin_make
source devel/setup.sh
rosrun bagextracter imageSaver  < image_type >  < /path/to/save/image >
```

## 在ROS 2环境中，capture_pcd_png.py是一个实用的Python脚本，专为捕获并保存点云和图像数据而设计。此工具能够便捷地从指定的ROS话题中抓取一次数据，将其分别保存为一张PNG图像文件和一个PCD点云文件。要运行此脚本，您的系统需配备Python 3环境。
使用方式如下:
1.定位到工作目录： 首先，通过命令行进入capture_pcd_png.py脚本所在的目录：
```bash
cd ros2/captureOnce
./capture_pcd_png.py </your_image_topic> </your_pointcloud_topic> <./your_save_path='./output'>
```
2.执行脚本保存数据： 接着，运行脚本并指定图像话题、点云话题以及保存路径（可选，默认为./output）：
```bash
./capture_pcd_png.py <your_image_topic> <your_pointcloud_topic> [./your_save_path='./output']
-- [./your_save_path='./output']: （可选项）指定保存文件的目录路径，默认为当前目录下的output文件夹。
