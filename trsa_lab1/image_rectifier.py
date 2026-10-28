import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import yaml
import os
import ament_index_python
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

class ImageRectifier(Node):
    def __init__(self):
        super().__init__('image_rectifier')
        
        self.publisher_rectified_image = self.create_publisher(Image, '/camera/image_rect', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cv_bridge = CvBridge()

        # Load calibration data from ost.yaml
        calibration_path = os.path.join(
            ament_index_python.get_package_share_directory('trsa_lab1'), 
            'calibration', 
            'ost.yaml'
        )
        self.load_camera_calibration(calibration_path)
        
        # Initialize camera model with loaded calibration parameters
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info) 

        self.get_logger().info(f"Calibration loaded from: {calibration_path}")

    def load_camera_calibration(self, calibration_path):
        """Loads camera calibration parameters from a YAML file."""
        with open(calibration_path, 'r') as file:
            self.camera_info = CameraInfo()
            calibration_data = yaml.safe_load(file)
            
            self.camera_info.width = calibration_data['image_width']
            self.camera_info.height = calibration_data['image_height']
            self.camera_info.distortion_model = calibration_data['distortion_model']
            
            self.camera_info.d = calibration_data['distortion_coefficients']['data']
            self.camera_info.k = calibration_data['camera_matrix']['data']
            
            self.camera_info.r = calibration_data['rectification_matrix']['data']
            self.camera_info.p = calibration_data['projection_matrix']['data']
    
    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rectified_image = cv_image.copy()
        
        # Apply camera model rectification
        self.camera_model.rectifyImage(cv_image, rectified_image) 
        
        rect_msg = self.cv_bridge.cv2_to_imgmsg(rectified_image, encoding='bgr8')
        rect_msg.header = msg.header 
        
        self.publisher_rectified_image.publish(rect_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageRectifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
