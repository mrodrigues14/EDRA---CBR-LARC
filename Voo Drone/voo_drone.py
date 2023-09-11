import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from std_msgs.msg import Float32MultiArray  # Importar para receber as coordenadas do objeto


class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

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

        self.target_height = -5.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_reached_target_height = False
        self.vehicle_local_position = VehicleLocalPosition()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.last_log_time = time.time()
        self.log_interval = 0.5

        self.has_armed = False
        self.has_engaged_offboard_mode = False
        self.has_published_position = False
        self.position_setpoint_count = 0
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "z": -5.0},
            {"x": 5.0, "y": 0.0, "z": -5.0},
            {"x": 5.0, "y": 2.0, "z": -5.0},
            {"x": 1.0, "y": 2.0, "z": -5.0},
            {"x": 1.0, "y": 4.0, "z": -5.0},
            {"x": 5.0, "y": 4.0, "z": -5.0},
            {"x": 5.0, "y": 6.0, "z": -5.0},
            {"x": 1.0, "y": 6.0, "z": -5.0},
            {"x": 0.0, "y": 0.0, "z": -5.0},
        ]
        self.current_waypoint_index = 0

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
        self.get_logger().info(f"Current coordinates: X: {msg.x:.3f}, Y: {msg.y:.3f}, Z: {msg.z:.3f}")

        current_waypoint = self.waypoints[self.current_waypoint_index]

        # Se alcançou o waypoint atual
        if abs(msg.x - current_waypoint["x"]) < 0.1 and abs(msg.y - current_waypoint["y"]) < 0.1 and abs(
                msg.z - current_waypoint["z"]) < 0.1:
            self.current_waypoint_index += 1  # Move para o próximo waypoint

            if self.current_waypoint_index >= len(self.waypoints):  # Se todos os waypoints foram alcançados
                self.land()  # Aterra o drone e termina a missão
            else:  # Se ainda há waypoints a serem alcançados
                self.publish_position_setpoint(current_waypoint["x"], current_waypoint["y"],
                                               current_waypoint["z"])  # Publica o próximo waypoint

        #if msg.z <= self.target_height and not self.has_reached_target_height:
            # self.has_reached_target_height = True
            # self.land()

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

        current_waypoint = self.waypoints[self.current_waypoint_index]
        self.publish_position_setpoint(current_waypoint["x"], current_waypoint["y"], current_waypoint["z"])

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
