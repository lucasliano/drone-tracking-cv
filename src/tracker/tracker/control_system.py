import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleLocalPosition, OffsetMsg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from tracker.drone_model import DroneModel
        


class DroneControlSystem(Node):
    def __init__(self):
        super().__init__('DroneControlSystem')
        # --- Control System Variables ---
        self.model = DroneModel()
        self.fs = 10
        self.yaw_offset = 0
        self.pitch_offset = 0
        self.distance_to_targe = 0


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
            topic       = "/tracker/control_system/setpoint",
            qos_profile = qos_profile #10
        )

        # --- Inicializamos los subscribers ---
        self.pos_sub = self.create_subscription(
            msg_type    = VehicleLocalPosition,
            topic       = '/fmu/out/vehicle_local_position',
            callback    = self.local_pos_callback,
            qos_profile = qos_profile
        )

        self.offset_sub = self.create_subscription(
            msg_type    = OffsetMsg,
            topic       = '/tracker/opencv/offset_to_center',
            callback    = self.sensor_callback,
            qos_profile = 10
        )

        # --- Configuración TASK 1 ---
        self.timer_main_task = self.create_timer(1/self.fs , self.main_task)

    def sensor_callback(self, msg):
        self.yaw_offset = msg.yaw
        self.pitch_offset = msg.pitch
        self.distance_to_targe = msg.distance

    def local_pos_callback(self, msg):
        '''
        Este es el callback del subscriber para el topic '/fmu/out/vehicle_local_position'.
        Lo único que hace es meterlo en una fifo.
        '''
        self.model.pos.x = round(msg.x, 3)
        self.model.pos.y = round(msg.y, 3)
        self.model.pos.z = -1 * round(msg.z, 3)

        self.model.vel.x = round(msg.vx, 3)
        self.model.vel.y = round(msg.vy, 3)
        self.model.vel.z = -1 * round(msg.vz, 3)

        self.model.acc.x = round(msg.ax, 3)
        self.model.acc.y = round(msg.ay, 3)
        self.model.acc.z = -1 * round(msg.az, 3)

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
        self.get_logger().info(f"Desired X: {round(x,3)} - Z: {round(z,3)} - Yaw: {round(yaw,3)} ")
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(-z)] 
        msg.yaw = float(yaw)  # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.setpoint_publisher.publish(msg)

    def main_task(self):
        
        self.model.update_setpoints(
            self.yaw_offset,
            self.pitch_offset,
            self.distance_to_targe,
        )

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