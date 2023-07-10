# Imports que vienen en el ejemplo
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import TrajectorySetpoint

# Imports personalizados
import os
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
import numpy as np
import time
from tracker.drone_model import DroneModel


class DroneJoystick(Node):

    MSG = """
    Control Your Drone!
    ---------------------------
    Moving around:
    q   w   e
    a   s   d
    z   x


    All movements are made by incrementing a position unit.


    w/s : Go forward/backwards
    a/d : Move Left/right
    z/x : Increase/Decrease altitude
    q/e : Increase/Decrease Yaw angle

    space : Switch mode (Joystick/Control_System)

    CTRL-C to quit
    """



    def __init__(self):
        super().__init__('DroneJoystick')
        self.model = DroneModel(plot=False)

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
            qos_profile = qos_profile
        )
        self.mode_publisher = self.create_publisher(
            msg_type    = Bool,
            topic       = "/tracker/joystick/mode",
            qos_profile = qos_profile
        )

        # Variable de configuración del modo de uso.
        self.joystick_mode = True

        # We request a takeoff
        self.model.pos_setpoint.x = 0
        self.model.pos_setpoint.y = 0
        self.model.pos_setpoint.z = 2
        self.model.heading = 0

        self.publish_trajectory_setpoint(
            x   = self.model.pos_setpoint.x,
            y   = self.model.pos_setpoint.y,
            z   = self.model.pos_setpoint.z,
            yaw = self.model.heading,
        )
        self.get_logger().info("Takeoff Requested")


        # TODO: Se deben hacer tareas que no sean bloqueantes. No hay scheduler.
        # --- Configuración TASK 1 ---
        self.timer_main_task = self.create_timer(0.1 , self.main_task)

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
        self.setpoint_publisher.publish(msg)

    def main_task(self):
        os.system('clear')
        if self.joystick_mode:
            print(self.MSG)

            #TODO: Cambiar a algo mejor..
            key = input('Select key and then press enter:')
            if key == 'w':
                # Go Forward
                self.model.pos_setpoint.x = self.model.pos_setpoint.x + 0.5
            elif key == 's':
                # Go Backward
                self.model.pos_setpoint.x = self.model.pos_setpoint.x - 0.5
            elif key == 'a':
                # Go Left
                self.model.pos_setpoint.y = self.model.pos_setpoint.y - 0.5
            elif key == 'd':
                # Go Right
                self.model.pos_setpoint.y = self.model.pos_setpoint.y + 0.5
            elif key == 'z':
                # Go Up
                self.model.pos_setpoint.z = self.model.pos_setpoint.z + 0.5
            elif key == 'x':
                # Go Down
                self.model.pos_setpoint.z = self.model.pos_setpoint.z - 0.5
            elif key == 'q':
                # Turn CCW
                self.model.yaw_setpoint = self.model.yaw_setpoint - np.pi/12
            elif key == 'e':
                # Turn CW
                self.model.yaw_setpoint = self.model.yaw_setpoint + np.pi/12
            elif key == ' ':
                # # Takeoff
                # self.model.pos_setpoint.x = 0
                # self.model.pos_setpoint.y = 0
                # self.model.pos_setpoint.z = 2
                # self.model.yaw_setpoint = np.pi/2
                
                self.joystick_mode = False
                msg = Bool()
                msg.data = bool(self.joystick_mode)
                self.mode_publisher.publish(msg)

            self.publish_trajectory_setpoint(
                x   = self.model.pos_setpoint.x,
                y   = self.model.pos_setpoint.y,
                z   = self.model.pos_setpoint.z,
                yaw = self.model.yaw_setpoint,
            )
        else:
            input('Control system is Active. Press Enter to enter joystick mode.')
            self.joystick_mode = True
            msg = Bool()
            msg.data = bool(self.joystick_mode)
            self.mode_publisher.publish(msg)

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
