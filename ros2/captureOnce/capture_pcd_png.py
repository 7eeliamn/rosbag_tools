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
import struct




class ImageSaver(Node):
    def __init__(self, png_subscription_topic, pcd_subscription_topic, output_dir):
        super().__init__('pcd_png_saver')
        self.png_subscription = self.create_subscription(
            Image,
            png_subscription_topic,    #'/image_raw/video7',  # 替换Image topic
            self.save_image,
            10
        )
        self.pcd_subscription = self.create_subscription(
            PointCloud2,
            pcd_subscription_topic,#      '/rslidar_points_main',  # 这里替换为你的PointCloud2消息的topic名称
            self.save_pointcloud_to_pcd_bin,
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

    def save_pointcloud_to_pcd_acs(self, msg):
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
    
    def save_pointcloud_to_pcd_bin(self, msg):
        # 将PointCloud2消息转换为PCD格式
        if self.save_condition_pcd:
            # 读取点云数据为结构化数组，准备写入二进制文件
            points_structured = pc2.read_points(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True)

            output_file = os.path.join(self.output_dir, f'{self.index}.pcd')
            with open(output_file, 'wb') as pcd_file:
                # 写入PCD文件头信息
                pcd_file.write(b'# .PCD v.7 - Point Cloud Data file format\n')
                pcd_file.write(b'VERSION .7\n')
                pcd_file.write(b'FIELDS x y z intensity\n')
                pcd_file.write(b'SIZE 4 4 4 4\n')
                pcd_file.write(b'TYPE F F F F\n')
                pcd_file.write(b'COUNT 1 1 1 1\n')
                pcd_file.write(b'WIDTH ')
                pcd_file.write(str(len(list(points_structured))).encode('utf-8'))
                pcd_file.write(b'\nHEIGHT 1\n')
                pcd_file.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
                pcd_file.write(b'POINTS ')
                pcd_file.write(str(len(list(points_structured))).encode('utf-8'))
                pcd_file.write(b'\nDATA binary\n')

                # 直接写入点云数据的二进制形式
                for point in points_structured:
                    # 修改这里以正确访问structured array的字段
                    pcd_file.write(struct.pack('ffff', point['x'], point['y'], point['z'], point['intensity']))

            print("PCD (binary) saved")
            self.save_condition_pcd = False
            
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
def main(png_subscription_topic='/image',pcd_subscription_topic='/points',output_dir='./output'):
    rclpy.init(args=None)
    image_saver = ImageSaver(png_subscription_topic, pcd_subscription_topic, output_dir)
    rclpy.spin(image_saver)
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        print("Usage: python3 script.py <png_topic> <pcd_topic> <output_directory>")
        print("   or: python3 script.py <png_topic> <pcd_topic> , default save in file: ./output")
        sys.exit(1)

    png_topic = sys.argv[1]
    pcd_topic = sys.argv[2]
    output_directory='./output'

    if len(sys.argv)==4:
        output_directory = sys.argv[3]

    main(png_topic, pcd_topic, output_directory)
