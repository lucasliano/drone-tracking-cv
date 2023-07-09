# Imports que vienen en el ejemplo
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleLocalPosition




# Imports personalizados
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from tracker.drone_model import DroneModel
        


class DroneControlSystem(Node):
    def __init__(self):
        super().__init__('DroneControlSystem')
        # --- Control System Variables ---
        self.model = DroneModel()
        self.fs = 1000
        self.target_center_offset = {
            'x':        0,
            'y':        0,
        }
        self.target_depth = 0


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

        # --- Inicializamos los subscribers ---
        self.pos_sub = self.create_subscription(
            msg_type    = VehicleLocalPosition,
            topic       = '/fmu/out/vehicle_local_position',
            callback    = self.local_pos_callback,
            qos_profile = qos_profile
        )

        # FIXME: Hay que cambiar el msg_type de ambos subs!
        self.center_sub = self.create_subscription(
            msg_type    = Bool,
            topic       = '/tracker/opencv/offset_to_center',
            callback    = self.local_pos_callback,
            qos_profile = qos_profile
        )
        self.depth_sub = self.create_subscription(
            msg_type    = Bool,
            topic       = '/tracker/opencv/depth',
            callback    = self.local_pos_callback,
            qos_profile = qos_profile
        )

        # --- Configuración TASK 1 ---
        self.timer_main_task = self.create_timer(1/self.fs , self.main_task)

    def center_callback(self, msg):
        pass
        self.target_center_offset['x'] = 0
        self.target_center_offset['y'] = 0

    def depth_callback(self, msg):
        pass
        self.target_depth = 0

    def local_pos_callback(self, msg):
        '''
        Este es el callback del subscriber para el topic '/fmu/out/vehicle_local_position'.
        Lo único que hace es meterlo en una fifo.
        '''
        self.model.pos[0] = round(msg.x, 3)
        self.model.pos[1] = round(msg.y, 3)
        self.model.pos[2] = -1 * round(msg.z, 3)

        self.model.vel[0] = round(msg.vx, 3)
        self.model.vel[1] = round(msg.vy, 3)
        self.model.vel[2] = -1 * round(msg.vz, 3)

        self.model.acc[0] = round(msg.ax, 3)
        self.model.acc[1] = round(msg.ay, 3)
        self.model.acc[2] = -1 * round(msg.az, 3)

        self.model.heading = round(msg.heading, 3)

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
        
        self.model.update_setpoints(self.target_center_offset, self.target_depth)

        self.publish_trajectory_setpoint(
            x   = self.model.pos_setpoint.x,
            y   = self.model.pos_setpoint.y,
            z   = self.model.pos_setpoint.z,
            yaw = self.model.yaw_setpoint,
        )
        

def main(args=None):
    rclpy.init(args=args)
    print("Starting DroneControlSystem node...\n")
    node = DroneControlSystem()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()