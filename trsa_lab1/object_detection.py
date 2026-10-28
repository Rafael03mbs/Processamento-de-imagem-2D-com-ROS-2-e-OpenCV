import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
import cv2
from cv_bridge import CvBridge
import numpy as np

# Área mínima (em píxeis) para um contorno ser considerado objeto. Elimina ruído.
MIN_AREA = 1000

# Kernel do fecho morfológico: tapa buracos pequenos dentro das máscaras de cor.
KERNEL = np.ones((15, 15), np.uint8)

# Nome especial que, se estiver em 'valid_targets', desliga o filtro (aceita tudo).
ACCEPT_ALL = 'ALL'


def build_color_ranges(sat_min, val_min):
    """Devolve {nome_cor: (lista_de_faixas_HSV, cor_BGR_para_desenhar)}.

    Cada cor tem uma LISTA de faixas porque o vermelho está nos dois extremos
    do círculo de hue do OpenCV (0-179): precisa de duas faixas (0-10 e 170-179).
    As faixas não se sobrepõem nem deixam buracos entre cores vizinhas.
    sat_min / val_min definem quão "viva" e "clara" uma cor tem de ser; com
    uma câmara real convém baixá-los (papel e luz alteram saturação e brilho).
    """
    def faixa(h_min, h_max):
        return (np.array([h_min, sat_min, val_min]), np.array([h_max, 255, 255]))

    return {
        "Vermelho":      ([faixa(0, 10), faixa(170, 179)], (0, 0, 255)),
        "Laranja":       ([faixa(11, 24)],                 (0, 165, 255)),
        "Amarelo":       ([faixa(25, 35)],                 (0, 255, 255)),
        "Amarelo-Verde": ([faixa(36, 45)],                 (0, 255, 200)),
        "Verde":         ([faixa(46, 80)],                 (0, 255, 0)),
        "Azul":          ([faixa(81, 130)],                (255, 0, 0)),
        "Violeta":       ([faixa(131, 169)],               (255, 0, 130)),
    }


def classify_shape(contorno):
    """Classifica a forma de um contorno. Devolve o nome da forma ou None.

    Critérios (todos invariantes à rotação no plano da imagem):
      - solidez = área / área do convex hull. Formas côncavas (estrela) têm
        solidez baixa; as convexas (triângulo, retângulo, círculo) ficam perto de 1.
      - nº de vértices após approxPolyDP (3 -> triângulo, 4 -> retângulo).
      - redondeza = área / área do menor círculo que envolve o contorno.
        Um círculo dá ~1; um quadrado ~0.64; um triângulo ~0.4.
    """
    area = cv2.contourArea(contorno)
    perimetro = cv2.arcLength(contorno, True)
    esquinas = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)

    area_hull = cv2.contourArea(cv2.convexHull(contorno))
    solidez = area / area_hull if area_hull > 0 else 0

    _, raio = cv2.minEnclosingCircle(contorno)
    redondeza = area / (np.pi * raio * raio) if raio > 0 else 0

    if solidez < 0.8 and len(esquinas) >= 6:
        return "Estrela"
    if len(esquinas) == 3:
        return "Triangulo"
    if len(esquinas) == 4:
        return "Retangulo"
    if redondeza > 0.75:
        return "Circulo"
    return None  # forma desconhecida: ignorada (nunca é rotulada nem contada)


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Topic de entrada configurável: /camera/image_rect por defeito, mas pode
        # usar-se /camera/image_raw para uma webcam/gravação sem calibração.
        # NUNCA /camera/image_processed (o Canny remove a cor).
        self.declare_parameter('image_topic', '/camera/image_rect')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.sub_image = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.pub_image = self.create_publisher(Image, '/object_detection/image', 10)
        self.pub_count = self.create_publisher(Int32, '/object_detection/count', 10)
        self.pub_alignment = self.create_publisher(String, '/object_detection/alignment', 10)

        # Alignment parameters
        self.declare_parameter('target_x', 0.5)
        self.declare_parameter('target_y', 0.5)
        self.declare_parameter('tol_x', 0.05)
        self.declare_parameter('tol_y', 0.05)

        # Combinações "Forma Cor" que contam como alvo válido. Qualquer outro objeto
        # detetado (distrator) é ignorado. Pode mudar-se em runtime:
        #   ros2 param set /object_detection valid_targets "['Circulo Azul']"
        # Com ['ALL'] aceita todas as combinações detetadas.
        self.declare_parameter('valid_targets', [
            'Circulo Azul',
            'Retangulo Vermelho',
            'Triangulo Amarelo',
            'Estrela Verde',
        ])

        # Limiares mínimos de saturação e brilho das máscaras HSV.
        self.declare_parameter('sat_min', 100)
        self.declare_parameter('val_min', 100)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Retrieve alignment parameters
        tx = self.get_parameter('target_x').get_parameter_value().double_value
        ty = self.get_parameter('target_y').get_parameter_value().double_value
        tol_x = self.get_parameter('tol_x').get_parameter_value().double_value
        tol_y = self.get_parameter('tol_y').get_parameter_value().double_value

        # Os parâmetros são lidos a cada frame para poderem mudar em runtime.
        valid_targets = set(
            self.get_parameter('valid_targets').get_parameter_value().string_array_value)
        aceitar_todos = ACCEPT_ALL in valid_targets
        sat_min = self.get_parameter('sat_min').get_parameter_value().integer_value
        val_min = self.get_parameter('val_min').get_parameter_value().integer_value

        # Convert relative target and tolerance values to pixel coordinates
        altura, largura, _ = cv_image.shape
        centro_alvo_x = int(largura * tx)
        centro_alvo_y = int(altura * ty)
        margem_x = int(largura * tol_x)
        margem_y = int(altura * tol_y)

        # HSV calculado ANTES de desenhar nada, para as anotações não influenciarem a deteção.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Draw target region
        cv2.rectangle(cv_image, (centro_alvo_x - margem_x, centro_alvo_y - margem_y),
                      (centro_alvo_x + margem_x, centro_alvo_y + margem_y), (255, 255, 255), 2)
        cv2.circle(cv_image, (centro_alvo_x, centro_alvo_y), 4, (255, 255, 255), -1)

        maior_area = 0
        melhor_alvo = None  # (cx, cy, (x, y, w, h)) do alvo válido com maior área
        contador = 0

        cores = build_color_ranges(sat_min, val_min)

        for nome_cor, (faixas, cor_bgr) in cores.items():
            # Junta todas as faixas da cor (o vermelho tem duas) numa só máscara
            # ANTES do fecho morfológico, para fechar buracos na máscara completa.
            mask = None
            for lim_inf, lim_sup in faixas:
                parcial = cv2.inRange(hsv, lim_inf, lim_sup)
                mask = parcial if mask is None else cv2.bitwise_or(mask, parcial)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

            contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contorno in contornos:
                area = cv2.contourArea(contorno)
                if area < MIN_AREA:
                    continue

                # 1) Deteção: que forma é?
                forma = classify_shape(contorno)
                if forma is None:
                    continue
                etiqueta = f"{forma} {nome_cor}"

                # 2) Filtro: só combinações cor+forma válidas passam. Um objeto que
                #    cumpre só um dos critérios (distrator) não é desenhado nem contado.
                if not aceitar_todos and etiqueta not in valid_targets:
                    continue

                # Centróide do objeto válido
                M = cv2.moments(contorno)
                x, y, w, h = cv2.boundingRect(contorno)
                if M["m00"] != 0:
                    obj_cx = int(M["m10"] / M["m00"])
                    obj_cy = int(M["m01"] / M["m00"])
                else:
                    obj_cx, obj_cy = x + w // 2, y + h // 2

                # Draw bounding box, centre and label (para TODOS os alvos válidos)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), cor_bgr, 3)
                cv2.circle(cv_image, (obj_cx, obj_cy), 5, cor_bgr, -1)
                cv2.putText(cv_image, etiqueta, (x, max(y - 10, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, cor_bgr, 2)

                contador += 1

                # Track the largest valid target (qualquer combinação válida)
                if area > maior_area:
                    maior_area = area
                    melhor_alvo = (obj_cx, obj_cy, (x, y, w, h))

        # Visual Alignment logic
        estado_alinhamento = "NO_TARGET"
        texto_offset = "Offset: --"

        if melhor_alvo is not None:
            obj_cx, obj_cy, (bx, by, bw, bh) = melhor_alvo

            # Distingue o alvo selecionado dos restantes: moldura branca extra,
            # centro vermelho maior e linha até à posição alvo.
            cv2.rectangle(cv_image, (bx - 6, by - 6), (bx + bw + 6, by + bh + 6), (255, 255, 255), 2)
            cv2.circle(cv_image, (obj_cx, obj_cy), 8, (0, 0, 255), -1)
            cv2.line(cv_image, (centro_alvo_x, centro_alvo_y), (obj_cx, obj_cy), (0, 255, 255), 2)

            # Calculate offsets relative to target center
            dx = obj_cx - centro_alvo_x
            dy = obj_cy - centro_alvo_y

            # Normalize offsets based on tolerance margins
            norm_x = abs(dx) / margem_x if margem_x > 0 else 0
            norm_y = abs(dy) / margem_y if margem_y > 0 else 0

            if norm_x <= 1.0 and norm_y <= 1.0:
                estado_alinhamento = "ALIGNED"
            else:
                # Determine alignment state, prioritizing horizontal alignment
                if norm_x >= norm_y:
                    estado_alinhamento = "RIGHT" if dx > 0 else "LEFT"
                else:
                    estado_alinhamento = "BELOW" if dy > 0 else "ABOVE"

            texto_offset = f"Offset: dx={dx}px dy={dy}px"

        # Estado, offsets e contagem SÓ DEPOIS da lógica e FORA do if: aparecem
        # sempre, incluindo quando não há alvo (NO_TARGET / contagem 0).
        cv2.putText(cv_image, f"State: {estado_alinhamento}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(cv_image, texto_offset, (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(cv_image, f"Valid targets: {contador}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        msg_estado = String()
        msg_estado.data = estado_alinhamento
        self.pub_alignment.publish(msg_estado)

        # Publish final annotated image
        msg_out = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        msg_out.header = msg.header
        self.pub_image.publish(msg_out)

        # Publish object count (alvos válidos neste frame; 0 se não houver)
        msg_count = Int32()
        msg_count.data = contador
        self.pub_count.publish(msg_count)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
