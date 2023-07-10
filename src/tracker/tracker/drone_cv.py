import os
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffsetMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.get_logger().info('Init Computer Vision')

        qos_profile = 10

        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image', 
            self.image_callback, 
            qos_profile
        )
        self.image_sub # prevent unused variable warning

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            qos_profile
        )
        self.depth_sub # prevent unused variable warning

        self.offset_pub = self.create_publisher(
            OffsetMsg,
            '/tracker/opencv/offset_to_center',
            qos_profile
        )

        # FIFOS
        self.color_fifo = []
        self.depth_fifo = []

        # --- Configuración TASK 1 ---
        self.timer_task = self.create_timer(0.01 , self.process_images)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Used to vectorize funcions
        self.vmap = np.vectorize(self.map)

        # Plot
        self.fig, self.axs = plt.subplots(1,2)
        plt.show(block=False)
        self.axs[0].imshow(np.arange(100).reshape((10,10)), cmap=plt.cm.get_cmap('inferno'))
        _sample_fig = self.axs[1].imshow(np.arange(100).reshape((10,10)) * 3.55/100, cmap=plt.cm.get_cmap('inferno'))
        self.fig.colorbar(_sample_fig, use_gridspec=False, cax=make_axes_locatable(self.axs[1]).append_axes("right", size="5%", pad=0.05))
        plt.pause(1)

    def map(self, value):
        # input [0.2, 3] - [0.7, 3.5]m

        # Se aplica un mapeo considerando que un input de 1 representa 1.5m, y un input de 3 representa 3.5m
        # 3.5m <= 3.0f
        # 1.5m <= 1.0f
        return value * 1 + 0.5 

    def image_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        self.color_fifo.append(current_frame)

    def depth_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='32FC1')
        self.depth_fifo.append(current_frame)

    def process_images(self):
        if len(self.color_fifo) > 1 and len(self.depth_fifo):
            # Sacamos las dos imagenes de la fifo
            current_color = self.color_fifo.pop()
            current_depth = self.depth_fifo.pop()

            # Procesamos las imagenes
            output_color, yaw, pitch, center = self.process_color(current_color)
            output_depth = self.process_depth(current_depth)

            # Publicamos las imagenes si se detectó un objeto
            if center:
                distance = output_depth[center[1],center[0]]
                self.get_logger().info(f'Yaw: {round(yaw,3)} - pitch: {round(pitch,3)} - Distance: {round(distance,3)}m')
            
                msg = OffsetMsg()
                msg.yaw = yaw
                msg.pitch = pitch
                msg.distance = distance
                self.offset_pub.publish(msg)

            # Plotteamos la camara de color y la de profundidad procesadas
            self.axs[0].clear()
            self.axs[1].clear()

            self.axs[0].imshow(output_color)
            self.axs[1].imshow(output_depth, cmap=plt.cm.get_cmap('inferno'))

            self.axs[0].set(title = 'Depth Image')
            self.axs[1].set(title = 'Normalized Image')
            plt.pause(0.1)


    def process_color(self, current_frame):
        img_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        mask = self.green_mask(img_hsv)
        mask = cv2.bilateralFilter(mask, 9, 75, 75)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, hierachy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if len(contours) != 0:
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > 250:
                moments = cv2.moments(max_contour)
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])

                # --- Calculo de yaw ---
                fov_x = 1.047198
                focal_lenght = 320/2 * np.tan(fov_x/2)
                alpha = np.arctan((320/2 - cx)/focal_lenght)
                yaw = alpha * 180/np.pi

                # --- Calculo de pitch ---
                fov_y = 1.047198
                focal_lenght = 240/2 * np.tan(fov_y/2)
                alpha = np.arctan((240/2 - cy)/focal_lenght)
                pitch = alpha * 180/np.pi

                # --- Procesamiento current_frame ---
                intermediate_frame = cv2.circle(current_frame, (cx, cy), 5, (0, 0, 255), -1)
                x,y,w,h = cv2.boundingRect(max_contour)
                output_frame = cv2.rectangle(intermediate_frame, (x,y), (x+w, y+h), (0, 0, 255), 3)

                return output_frame, yaw, pitch, (cx,cy)

        return current_frame, None, None, None


    def process_depth(self, current_frame):
        # Clippeamos la imagen
        normalized_img = np.clip(current_frame, 0.2, 3)

        # Aplicar la función map a cada valor del arreglo
        depth_img = self.vmap(normalized_img)

        return depth_img

    def green_mask(self, img_hsv):
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])
        mask = cv2.inRange(img_hsv, lower_green, upper_green)
        return mask


def main(args=None):
    try:
        rclpy.init(args=args)

        image_subscriber = ImageSubscriber()

        rclpy.spin(image_subscriber)

        image_subscriber.destroy_node()

        rclpy.shutdown()
    except KeyboardInterrupt:
        os._exit(0)

if __name__ == '__main__':
    main()
