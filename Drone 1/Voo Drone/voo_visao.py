import time

# Importações de controle
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from std_msgs.msg import Float32MultiArray

# Importações de visão
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov5.models.experimental import attempt_load
from yolov5.utils.torch_utils import select_device
from yolov5.utils.general import non_max_suppression



class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')
        self.get_logger().info("Nó de controle offboard inicializado")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # Assinatura para receber as coordenadas do objeto
        self.object_coordinates_subscriber = self.create_subscription(
            Float32MultiArray, '/object_coordinates', self.object_coordinates_callback, qos_profile)

        self.target_height = -5.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_reached_target_height = False
        self.vehicle_local_position = VehicleLocalPosition()

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.has_armed = False
        self.has_engaged_offboard_mode = False
        self.has_published_position = False
        self.position_setpoint_count = 0
        # Adicione variáveis para acompanhar as coordenadas do objeto
        self.object_detected = False
        self.object_x = 0
        self.object_y = 0

        self.position_setpoint_count = 0
        self.base_detected = False

        self.vision_node_activated = False

    def arm(self):
        if not self.has_armed:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info('Arm command sent')
            self.has_armed = True

    def engage_offboard_mode(self):
        if not self.has_engaged_offboard_mode:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info("Switching to offboard mode")
            self.has_engaged_offboard_mode = True

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.get_logger().info(f"Current coordinates: X: {msg.x:.3f}, Y: {msg.y:.3f}, Z: {msg.z:.3f}")
        if msg.z <= self.target_height and not self.main_node.vision_node_active:
            self.main_node.vision_node_active = True
            self.main_node.activate_vision_node()
            self.get_logger().info("Target height reached, activating vision node")

    # if msg.z <= self.target_height and not self.has_reached_target_height:
        #     self.has_reached_target_height = True
        #     self.land()

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
        self.get_logger().info("Missão bem-sucedida")
        rclpy.shutdown()

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]} (count: {self.position_setpoint_count + 1})")
        self.position_setpoint_count += 1

    def object_coordinates_callback(self, msg):
        self.object_x, self.object_y = msg.data
        self.object_detected = True

    def control_drone(self):
        # Verifique se o objeto foi detectado
        if self.object_detected:
            # Controle do drone para ir até o objeto
            if not self.base_detected:
                if not self.has_armed:
                    self.arm()
                if not self.has_engaged_offboard_mode:
                    self.engage_offboard_mode()

                # Ajuste a posição x e y do drone para ir em direção ao objeto
                drone_x = self.object_x
                drone_y = self.object_y

                # Mantenha a mesma altura
                drone_z = self.target_height

                self.publish_position_setpoint(drone_x, drone_y, drone_z)

                # Verifique se o drone está próximo o suficiente do objeto para pousar
                distance_to_object = ((self.vehicle_local_position.x - drone_x) ** 2 + (
                            self.vehicle_local_position.y - drone_y) ** 2) ** 0.5
                if distance_to_object < 0.5:  # Ajuste a distância conforme necessário
                    self.base_detected = True
                    self.land()
            else:
                self.land()  # Já detectou o objeto, então pouse

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0,
                                param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()
        self.engage_offboard_mode()
        self.arm()
        self.publish_position_setpoint(self.target_x, self.target_y, self.target_height)
        # self.control_drone()  # Chame a função de controle em vez de publicar a posição diretamente

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info("Nó de visão inicializado")
        self.publisher = self.create_publisher(Float32MultiArray, '/object_coordinates', 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/drone/camera/image_raw',
            self.image_callback,
            10
        )

        # Carregar o modelo treinado YOLOv5
        weights = 'yolov5s.pt'
        self.device = select_device('')  # Use uma GPU disponível, se disponível
        self.model = attempt_load(weights)
        self.model = self.model.to(self.device)
        self.stride = int(self.model.stride.max())  # Stride máximo para redimensionar corretamente as coordenadas

        # Configurações para supressão não máxima
        self.conf_threshold = 0.3
        self.iou_threshold = 0.4

        self.active = False

    def activate(self):
        self.active = True

    def image_callback(self, msg):
        if not self.main_node.vision_node_active:
            return

        self.get_logger().info('Image callback called')
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(frame)

    def publish_coordinates(self, coordinates):
        msg = Float32MultiArray()
        msg.data = coordinates
        self.publisher.publish(msg)

    def process_frame(self, frame):
        # Detecção de objetos com o YOLOv5
        img = torch.from_numpy(frame).to(self.device)
        img = img.float() / 255.0
        img = img.permute(2, 0, 1).unsqueeze(0)

        pred = self.model(img)[0]
        pred = non_max_suppression(pred, self.conf_threshold, self.iou_threshold)[0]

        if pred is not None and len(pred):
            for det in pred:
                x1, y1, x2, y2, conf, cls = det.tolist()
                label = f'{self.model.names[int(cls)]} {conf:.2f}'

                # Calcular e exibir o centro do objeto
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # Publicar as coordenadas do objeto detectado no tópico personalizado (ROS)
                object_coordinates = Float32MultiArray()
                object_coordinates.data = [float(center_x), float(center_y)]
                self.publish_coordinates(object_coordinates.data)

                if self.model.names[int(cls)] == 'base':
                    self.get_logger().info(f'Base detected at coordinates: {object_coordinates.data}')
                else:
                    self.get_logger().info('No base detected.')

        cv2.imshow('YOLOv5 Object Detection', frame)
        cv2.waitKey(1)

    def log_info(self, info):
        self.get_logger().info(info)


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.offboard_control_node = OffboardControl()
        self.offboard_control_node.main_node = self
        self.vision_node = VisionNode()
        self.vision_node.main_node = self
        self.vision_node_active = False

    def activate_vision_node(self):
        self.vision_node_active = True

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()