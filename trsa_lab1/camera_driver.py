import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Configure video capture (MJPG at 1280x720, 5fps)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 5) 
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.is_video_file = False
        if not self.cap.isOpened():
            self.get_logger().warn('Camera not detected. Using local video file fallback.')
            # Caminho absoluto no share/ do pacote: não depende da pasta onde corres o comando.
            video_path = os.path.join(get_package_share_directory('trsa_lab1'), 'video', 'test3.mp4')
            self.cap = cv2.VideoCapture(video_path)
            self.is_video_file = True

        self.get_logger().info(
            f"Initialized Camera: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} "
            f"@ {self.cap.get(cv2.CAP_PROP_FPS)} FPS"
        )

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.is_video_file:
                # Fim do vídeo: volta ao início para o stream não parar.
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        
        # Convert OpenCV frame to ROS 2 Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Liberta a câmara mesmo com Ctrl-C, senão pode ficar "ocupada".
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
