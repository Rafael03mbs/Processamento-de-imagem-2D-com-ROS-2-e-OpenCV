import rclpy # Base do ROS 2
import rclpy.node # Classe do nó
from sensor_msgs.msg import Image # Tipo da mensagem recebida e publicada
import cv2 # OpenCV, aqui só preciso indiretamente, mas útil
from cv_bridge import CvBridge # Conversor de ROS Image <-> OpenCV


class ImageConvert(rclpy.node.Node): # Nó retificador
    def __init__(self):
        super().__init__('imageconvert') # Nome do nó

        self.sub_image_rect = self.create_subscription(Image, '/camera/image_rect', self.image_callback, 10)
        self.pub_image_processed = self.create_publisher(Image, '/camera/image_processed', 10)
        
        self.bridge = CvBridge() # Inicializa o conversor de imagens

    def image_callback(self, msg):
        # 0. Converte a mensagem recebida para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 1. Converte para Grayscale (como pede o guião)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 2. Desfoque Gaussiano com kernel 21x21 (como pede o guião)
        blur = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # 3. Filtro de Canny para detetar as arestas (o tal contorno branco em fundo preto)
        arestas = cv2.Canny(blur, 80, 100)
        
        # 4. Converte a imagem binária (mono8) de volta para uma mensagem ROS e publica!
        msg_processada = self.bridge.cv2_to_imgmsg(arestas, encoding='mono8')
        
        # 5. Copia o carimbo de tempo para não perder os metadados da foto
        msg_processada.header = msg.header
        
        self.pub_image_processed.publish(msg_processada)

def main(args=None):
    rclpy.init(args=args)
    node = ImageConvert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
