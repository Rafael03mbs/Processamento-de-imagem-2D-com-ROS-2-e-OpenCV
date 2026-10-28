import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader')

        # Declare and retrieve target topic parameter
        # (ex.: -p topic:=/camera/image_rect ou /object_detection/image)
        self.declare_parameter('topic', '/camera/image_raw')
        target_topic = self.get_parameter('topic').get_parameter_value().string_value

        # Parte 2 (ponto 3): o reader subscreve TAMBÉM a imagem processada,
        # sem ser preciso lançar outra instância do nó.
        self.declare_parameter('processed_topic', '/camera/image_processed')
        processed_topic = self.get_parameter('processed_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Cada subscrição tem a sua janela, com o nome do topic como título.
        self.subscription = self.create_subscription(
            Image,
            target_topic,
            lambda msg: self.show(msg, target_topic),
            10
        )
        self.subscription_processed = self.create_subscription(
            Image,
            processed_topic,
            lambda msg: self.show(msg, processed_topic),
            10
        )

    def show(self, msg, window_name):
        # 'passthrough' mantém o encoding original: bgr8 (cor) ou mono8 (Canny)
        # sem forçar conversões nem erros.
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Display the image window
        cv2.imshow(window_name, cv_image)
        # Sem waitKey a janela não atualiza: é ele que processa os eventos da GUI.
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
