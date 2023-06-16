# Imports que vienen en el ejemplo
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus


# Imports personalizados
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class BotController(Node):
    def __init__(self):
        super().__init__('PX4Controller')

        # --- Propiedades del drone ---
        self.armed = False



        # --- Definimos un perfil para settear la calidad de servicio ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # --- Inicializamos los publishers ---
        self.offboard_publisher = self.create_publisher(
            msg_type    = OffboardControlMode,
            topic       = "/fmu/in/offboard_control_mode",
            qos_profile = qos_profile #10
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            msg_type    = TrajectorySetpoint,
            topic       = "/fmu/in/trajectory_setpoint",
            qos_profile = qos_profile #10
        )
        self.vehicle_command_publisher = self.create_publisher(
            msg_type    = VehicleCommand,
            topic       = "/fmu/in/vehicle_command",
            qos_profile = qos_profile #10
        )

        # --- Inicializamos los subscribers ---
        self.status_sub = self.create_subscription(
            msg_type    = VehicleStatus,
            topic       = '/fmu/out/vehicle_status',
            callback    = self.vehicle_status_callback,
            qos_profile = qos_profile
        )


        # TODO: Se deben hacer tareas que no sean bloqueantes. No hay scheduler.
        # --- Configuración TASK 1 ---
        self.timer_task1 = self.create_timer(0.1 , self.task1)
        self._task1_counter = 0

        # --- Configuración TASK 2 ---
        self.timer_task2 = self.create_timer(10 , self.task2)
        self.pos = 1


    def task1(self):
        if (self._task1_counter == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.alive_signal()

        # stop the counter after reaching 11
        if (self._task1_counter < 11):
            self._task1_counter += 1

    def task2(self):
        if self.armed:
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = 0,
                z   = self.pos,
                yaw = 0,
            )
            self.pos = self.pos + 1


    def alive_signal(self):
        '''
        Esta función se ocupa de enviar un mensaje a modo de keep-alive. Esta función se debe ejecutar 
        con una frecuencia mayor a 2Hz para que el dron permanezca en modo offboard.
        '''
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_publisher.publish(msg)

    def vehicle_status_callback(self, msg):
        '''
        Este es el callback del subscriber.
        '''
        # TODO: handle NED->ENU transformation
        print(f"NAV_STATUS: {msg.nav_state} - Offboard Status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD} ")
        self.nav_state = msg.nav_state

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        '''
        Publica un nuevo setpoint en la trayectoria.
        Los valores que se pueden settear son:

        float32[3] position     # in meters
        float32[3] velocity     # in meters/second
        float32[3] acceleration # in meters/second^2
        float32[3] jerk         # in meters/second^3 (for logging only)

        float32 yaw             # euler angle of desired attitude in radians -PI..+PI
        float32 yawspeed        # angular velocity around NED frame z-axis in radians/second
        '''
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(-z)] 
        msg.yaw = float(yaw)  # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info("Trajectory Setpoint sent")
    
    def publish_vehicle_command(self, 
        command, 
        param1 = 0.0,
        param2 = 0.0,
        param3 = 0.0,
        param4 = 0.0,
        param5 = 0.0,
        param6 = 0.0,
        param7 = 0.0,
    ):  
        '''
        Publica un vehicle commands. Para ello se necesita el comando a publicar y los parámetros.

            command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)    
        '''
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command       # command ID
        msg.target_system = 1       # system which should execute the command
        msg.target_component = 1    # component which should execute the command, 0 for all components
        msg.source_system = 1       # system sending the command
        msg.source_component = 1    # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        '''
        Arm the vehicle.
        '''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")
        self.armed = True

    def disarm(self):
        '''
        Disarm the vehicle.
        '''
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")
        self.armed = True


def main(args=None):
    rclpy.init(args=args)
    print("Starting PX4Controller node...\n")
    px4_controller = PX4Controller()
    rclpy.spin(px4_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    px4_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
