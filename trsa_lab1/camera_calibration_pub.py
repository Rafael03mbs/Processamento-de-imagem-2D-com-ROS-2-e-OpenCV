import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import yaml
from image_geometry import PinholeCameraModel
import os
import ament_index_python
from cv_bridge import CvBridge 
import threading

from sensor_msgs.srv import SetCameraInfo

class CameraCalibrationPub(Node):

    def __init__(self):
        super().__init__('camera_calibration_pub_node')
        
        # Initialize CameraInfo to None
        self.camera_info = None
        self.camera_model = PinholeCameraModel()
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.frame_id = '/base_link'
        self.rate = self.create_rate(15.0)

        calibration_path = os.path.join(
            ament_index_python.get_package_share_directory('trsa_lab1'), 'calibration','ost.yaml')

        # Create publishers and subscribers
        self.image_pub_raw = self.create_publisher(Image, '/camera/image_raw', 1)

        #Spin in a separate thread to avoid blocking on rate.sleep() due to ROS2 execution model
        self.thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        self.thread.start()

        calibration_path = os.path.join(
            ament_index_python.get_package_share_directory('trsa_lab1'), 'calibration','ost.yaml')

        # Create publishers and subscribers
        self.image_pub_raw = self.create_publisher(Image, '/camera/image_raw', 1)

        img_idx = 0

        while rclpy.ok():
            image_path = os.path.join(
                ament_index_python.get_package_share_directory("trsa_lab1"), 'calibration', 'images', f'{img_idx}.jpeg')

            frame = cv2.imread(image_path, cv2.IMREAD_COLOR)

            if img_idx > 126:
                self.get_logger().warn('Calibrate Now !!!')
                img_idx -= 1
            
            else:
                ros_img_raw = self.br.cv2_to_imgmsg(frame, 'bgr8')
                ros_img_raw.header.frame_id = self.frame_id
                ros_img_raw.header.stamp = self.get_clock().now().to_msg()
                self.image_pub_raw.publish(ros_img_raw)
                self.rate.sleep()

            img_idx += 1

        self.get_logger().warn('Calibration finished...')

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationPub()
    node.thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()