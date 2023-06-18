# Imports que vienen en el ejemplo
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand

# Imports personalizados
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool


class PX4Comms(Node):
    def __init__(self):
        super().__init__('PX4Comms')

        # --- Propiedades del drone ---
        self.armed = False
        self.mode_joystick = True
        self.setpoints = []

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
        self.control_sub = self.create_subscription(
            msg_type    = TrajectorySetpoint,
            topic       = '/tracker/control_system/setpoint',
            callback    = self.control_system_callback,
            qos_profile = qos_profile
        )

        self.joystick_sub = self.create_subscription(
            msg_type    = TrajectorySetpoint,
            topic       = '/tracker/joystick/setpoint',
            callback    = self.joystick_callback,
            qos_profile = qos_profile
        )

        self.mode_sub = self.create_subscription(
            msg_type    = Bool,
            topic       = '/tracker/joystick/mode',
            callback    = self.mode_callback,
            qos_profile = qos_profile
        )

        # --- Configuración TASK 1 ---
        self.timer_task1 = self.create_timer(0.25 , self.task1) # Freq = 4Hz
        self._task1_counter = 0

        # --- Configuración TASK 2 ---
        self.timer_task2 = self.create_timer(0.1 , self.task2)


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
        if self.armed and len(self.setpoints) != 0:
            new_setpoint = self.setpoints.pop()
            self.trajectory_setpoint_publisher.publish(new_setpoint)
            self.get_logger().info("Trajectory Setpoint sent")

            
            # TODO: Esto iria en el joystick y en el control_system
            # self.publish_trajectory_setpoint(
            #     x   = new_setpoint.x,
            #     y   = new_setpoint.y,
            #     z   = new_setpoint.z,
            #     yaw = new_setpoint.yaw,
            # )

    def mode_callback(self, msg):
        '''
        Este es el callback del subscriber para el topic '/tracker/joystick/mode'.
        '''
        self.mode_joystick = bool(msg.data)
        self.get_logger().info(f"Mode Switched to: {'Joystick' if self.mode_joystick else 'Control System'}")

    def joystick_callback(self, msg):
        '''
        Este es el callback del subscriber para el topic '/tracker/joystick/setpoint'.
        '''
        if self.mode_joystick:
            self.setpoints.insert(0, msg)

    def control_system_callback(self, msg):
        '''
        Este es el callback del subscriber para el topic '/tracker/control_system/setpoint'.
        '''
        if not self.mode_joystick:
            self.setpoints.insert(0, msg)

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
    print("Starting PX4Comms node...\n")
    px4_controller = PX4Comms()
    rclpy.spin(px4_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    px4_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
