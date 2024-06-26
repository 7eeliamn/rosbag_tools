#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import cv2
import numpy as np
import os
import select
import termios
import sys
from threading import Thread




class ImageSaver(Node):
    def __init__(self, output_dir='./output'):
        super().__init__('pcd_png_saver')
        self.png_subscription = self.create_subscription(
            Image,
            '/image_raw/video7',  # 替换Image topic
            self.save_image,
            10
        )
        self.pcd_subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points_main',  # 这里替换为你的PointCloud2消息的topic名称
            self.save_pointcloud_to_pcd,
            10
        )
        # 文件保存路径
        self.output_dir=output_dir
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        # 文件序列
        self.index = 0
        # 初始时不保存
        self.save_condition_pcd = False  
        self.save_condition_png = False  
        self.keyboard_thread = Thread(target=self.keyboard_listener)
        self.keyboard_thread.start()

    def save_image(self, msg):
        if self.save_condition_png:
            # 将ROS 2 Image消息转换为OpenCV图像格式
            bridge = CvBridge()
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except cv2.error as e:
                self.get_logger().info(f'CvBridge failed: {e}')
                return
            # 保存图像为PNG格式
            output_file=os.path.join(self.output_dir, f'{self.index}.png')
            cv2.imwrite(output_file, cv_image)

            print("Image saved")
            self.save_condition_png =False
        # else:
        #     print("Waiting for Enter key to save PNG...")

    def save_pointcloud_to_pcd(self, msg):
        # 将PointCloud2消息转换为PCD格式
        if self.save_condition_pcd:
            points = list(pc2.read_points(msg, field_names=["x", "y", "z", "intensity"],skip_nans=True))
            output_file=os.path.join(self.output_dir, f'{self.index}.pcd')
            with open(output_file, 'w') as pcd_file:
                pcd_file.write(f"# .PCD v.7 - Point Cloud Data file format\n")
                pcd_file.write(f"VERSION .7\n")
                pcd_file.write(f"FIELDS x y z intensity\n")
                pcd_file.write(f"SIZE 4 4 4 4\n")
                pcd_file.write(f"TYPE F F F F\n")
                pcd_file.write(f"COUNT 1 1 1 1\n")
                pcd_file.write(f"WIDTH {len(points)}\n")
                pcd_file.write("HEIGHT 1\n")
                pcd_file.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                pcd_file.write("POINTS " + str(len(points)) + "\n")
                pcd_file.write("DATA ascii\n")
                for point in points:
                    pcd_file.write(f"{point[0]} {point[1]} {point[2]} {point[3]:.3f}\n")
            
            print("PCD saved")
            self.save_condition_pcd =False
        # else:
        #     print("Waiting for Enter key to save PCD...")
    def keyboard_listener(self):
            # 监听键盘输入
            while True:
                input_char = input("Press Enter to save PNG-PCD: ")
                print(f"state:{self.save_condition_png}--- {self.save_condition_pcd }")
                if input_char == '' and self.save_condition_png == False and self.save_condition_pcd == False:
                    self.save_condition_png = True 
                    self.save_condition_pcd = True
                    self.index += 1
                    print("saving...")
def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
