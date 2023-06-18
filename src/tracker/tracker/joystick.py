# Imports que vienen en el ejemplo
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleLocalPosition




# Imports personalizados
import os
# import keyboard
# import pygame
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
import numpy as np


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
        self.timer_main_task = self.create_timer(0.1 , self.main_task)

    def local_pos_callback(self, msg):
        '''
        Este es el callback del subscriber para el topic '/fmu/out/vehicle_local_position'.
        Lo único que hace es meterlo en una fifo.
        '''
        self.get_logger().error('No se esta printeando el local_pos_callback')
        return
        if self.pos_skip_counter == 0:
            self.get_logger().info(f'Position: [{round(msg.x,3)}, {round(msg.y,3)}, {-round(msg.z,3)}]m - Heading: [{round(msg.heading,3)}]rad')
            self.get_logger().info(f'Velocity: [{round(msg.vx,3)}, {round(msg.vy,3)}, {-round(msg.vz,3)}]m/s')
            self.get_logger().info(f'Acceleration: [{round(msg.ax,3)}, {round(msg.ay,3)}, {-round(msg.az,3)}]m/s²')
        self.pos_skip_counter = self.pos_skip_counter + 1
        self.pos_skip_counter = self.pos_skip_counter % 10

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
        print(self.MSG)

        #TODO: Cambiar a algo mejor.. nefasto todo..
        key = input('Select key and then press enter:')
        if key == 'w':
            # Go Forward
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = 1,
                z   = 2,
                yaw = 0,
            )
        elif key == 's':
            # Go Backward
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = -1,
                z   = 2,
                yaw = 0,
            )
        elif key == 'a':
            # Go Left
            self.publish_trajectory_setpoint(
                x   = -1,
                y   = 0,
                z   = 2,
                yaw = 0,
            )
        elif key == 'd':
            # Go Right
            self.publish_trajectory_setpoint(
                x   = 1,
                y   = 0,
                z   = 2,
                yaw = 0,
            )
        elif key == 'z':
            # Go Up
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = 0,
                z   = 3,
                yaw = 0,
            )
        elif key == 'x':
            # Go Down
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = 0,
                z   = 1,
                yaw = 0,
            )
        elif key == 'q':
            # Turn CCW
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = 0,
                z   = 2,
                yaw = np.pi/2,
            )
        elif key == 'e':
            # Turn CW
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = 0,
                z   = 2,
                yaw = -np.pi/2,
            )
        elif key == ' ':
            # Takeoff
            self.publish_trajectory_setpoint(
                x   = 0,
                y   = 0,
                z   = 2,
                yaw = 0,
            )
        # try:
        #     if keyboard.is_pressed('w'):
        #         print('w')
        #     elif keyboard.is_pressed('s'):
        #         print('s')
        #     elif keyboard.is_pressed('a'):
        #         print('a')
        #     elif keyboard.is_pressed('d'):
        #         print('d')
        #     elif keyboard.is_pressed('z'):
        #         print('z')
        #     elif keyboard.is_pressed('x'):
        #         print('x')
        #     elif keyboard.is_pressed('q'):
        #         print('q')
        #     elif keyboard.is_pressed('e'):
        #         print('e')
        #     elif keyboard.is_pressed('space'):
        #         print('space')
        # except Exception as ex:
        #     print('ex')
        #     pass



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
