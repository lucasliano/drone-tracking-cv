#!/bin/bash

export $(grep -v '^#' .env | xargs)

# Preguntamos qué comando se quiere ejecutar
echo "What to do?"
echo "1. Start MicroXRCEAgent and publish gazebo-ros topics."
echo "2. Launch example subscriber."
echo "3. Launch example publisher."
echo "4. Build ROS2 packages."
echo "5. Run QGroundControl."
echo "6. Interactive terminal on ROS2 container."
echo "7. Interactive terminal on PX4 container."
echo "q. Exit."

read -p "Select option: " -r -n 1 option
echo ""

case $option in
    1) {
        command="coproc ros2 run ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image@gz.msgs.Image; \
        coproc ros2 run ros_gz_bridge parameter_bridge /camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image; \
        coproc ros2 run ros_gz_bridge parameter_bridge /camera/depth_image@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked; \
        coproc ros2 run ros_gz_bridge parameter_bridge /camera/depth_image@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo; \
        MicroXRCEAgent udp4 -p 8888"
    };;

    2) command="ros2 run px4_ros_com sensor_combined_listener";;

    3) command="ros2 run px4_ros_com debug_vect_advertiser";;

    4) command="colcon build --symlink-install";;

    5) {
        qgc="../../QGroundControl.AppImage"

        # Chequeo que esté descargado QGroundControl
        if [ ! -x "${qgc}" ]; then
            wget -O "${qgc}" https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
            chmod 774 "${qgc}"
        fi

        # Ejecutar QGC y salir
        $qgc
        exit 0

    };;

    6) command="/bin/bash";;

    7) {
        command="/bin/bash"
        docker container exec -ti -u "$(id -un)" "${px4_cont_name}" /bin/bash -c "
            source /entrypoint.sh; \
            ${command};"
        exit 0
    };;

    *) {
        echo "Exiting..."
        exit 0
    };;
esac

# Ejecutamos dentro del contenedor el archivo entrypoint y además el comando seleccionado
docker container exec -ti -u "$(id -un)" "${ros2_cont_name}" /bin/bash -c "
    source /entrypoint.sh; \
    ${command};"