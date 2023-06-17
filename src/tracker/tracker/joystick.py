# Imports que vienen en el ejemplo
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleLocalPosition

from std_msgs import Bool



# Imports personalizados
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class DroneJoystick(Node):
    def __init__(self):
        super().__init__('DroneJoystick')

        # --- Definimos un perfil para settear la calidad de servicio ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # --- Inicializamos los publishers ---
        self.setpoint_publisher = self.create_publisher(
            msg_type    = TrajectorySetpoint,
            topic       = "/tracker/joystick/setpoint",
            qos_profile = qos_profile #10
        )
        self.mode_publisher = self.create_publisher(
            msg_type    = Bool,
            topic       = "/tracker/joystick/mode",
            qos_profile = qos_profile #10
        )

        # --- Inicializamos los subscribers ---
        self.pos_sub = self.create_subscription(
            msg_type    = VehicleLocalPosition,
            topic       = '/fmu/out/vehicle_local_position',
            callback    = self.local_pos_callback,
            qos_profile = qos_profile
        )
        self.pos_skip_counter = 0


        # TODO: Se deben hacer tareas que no sean bloqueantes. No hay scheduler.
        # --- Configuración TASK 1 ---
        self.timer_task1 = self.create_timer(0.1 , self.task1)

        # --- Configuración TASK 2 ---
        self.timer_task2 = self.create_timer(10 , self.task2)

    def local_pos_callback(self, msg):
        '''
        Este es el callback del subscriber para el topic '/fmu/out/vehicle_local_position'.
        Lo único que hace es meterlo en una fifo.
        '''
        if self.pos_skip_counter == 0:
            self.get_logger().info(f'Position: [{round(msg.x,3)}, {round(msg.y,3)}, {-round(msg.z,3)}]m - Heading: [{round(msg.heading,3)}]rad')
            self.get_logger().info(f'Velocity: [{round(msg.vx,3)}, {round(msg.vy,3)}, {-round(msg.vz,3)}]m/s')
            self.get_logger().info(f'Acceleration: [{round(msg.ax,3)}, {round(msg.ay,3)}, {-round(msg.az,3)}]m/s²')
        self.pos_skip_counter = self.pos_skip_counter + 1
        self.pos_skip_counter = self.pos_skip_counter % 10



    def task1(self):
        pass

    def task2(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    print("Starting DroneJoystick node...\n")
    joystick = DroneJoystick()
    rclpy.spin(joystick)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
