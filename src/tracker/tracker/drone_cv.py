import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.get_logger().info('Init Computer Vision')

        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image', 
            self.image_callback, 
            10
        )
        self.image_sub # prevent unused variable warning

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10
        )
        self.depth_sub # prevent unused variable warning

        self.distance_pub = self.create_publisher(
            String,
            '/control/distance',
            10
        )

        self.yaw_pub = self.create_publisher(
            Float32,
            '/tracker/cv/yaw',
            10
        )

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def image_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        self.process_image(current_frame)


    def process_image(self, frame):
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

                yaw_msg = Float32()
                yaw_msg.data = yaw
                self.yaw_pub.publish(yaw_msg)


    def green_mask(self, img_hsv):
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])
        mask = cv2.inRange(img_hsv, lower_green, upper_green)
        return mask
  

    def depth_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.process_depth(current_frame)
    

    # def process_depth(self, frame):
    #     depth_array = np.array(frame, dtype=np.float32)
    #     imgplot = plt.imshow(depth_array)
    #     plt.pause(1)
        # img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # mask = self.grey_mask(img_hsv)

        # # Calculate the mean color of the masked region
        # mean_color = tuple(int(value) for value in cv2.mean(img_hsv, mask=mask)[:3])

        # center_msg = String()
        # center_msg.data = str(mean_color)
        # self.center_pub.publish(center_msg)
    
    def process_depth(self, frame):
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
                dist, angle = self.calculate_distance(mask, cx, cy)
                self.get_logger().info(f"dist = {dist} | angle = {angle}")


    def calculate_distance(image, pixel_x, pixel_y):
        # Convertir la imagen a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Calcula la distancia mínima y máxima basada en los valores de clip
        clip_near = 0.05  # Distancia mínima de clip en metros
        clip_far = 3.0  # Distancia máxima de clip en metros

        # Calcula la distancia entre la cámara y el objeto en función de la resolución y el FOV
        fov_horizontal = 1.047198  # Campo de visión horizontal en radianes
        image_width = image.shape[1]  # Ancho de la imagen en píxeles

        # Calcula el ángulo horizontal en radianes correspondiente al píxel seleccionado
        angle = (pixel_x / image_width - 0.5) * fov_horizontal

        # Calcula la distancia utilizando la fórmula trigonométrica
        distance = (clip_far - clip_near) / (2 * np.tan(fov_horizontal / 2)) * np.cos(angle) + (clip_near + clip_far) / 2

        return distance, angle


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
