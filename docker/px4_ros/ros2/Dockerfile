FROM ros:humble

# Instalar XRCE-DDs, siguiendo https://docs.px4.io/main/en/ros/ros2_comm.html#install-px4
RUN apt update && apt install -y \
    pip \
    git \
    wget \
    python3-colcon-common-extensions \
    && pip install -U empy pyros-genmsg setuptools \
    && apt remove pip \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "cd /home; \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git; \
    cd Micro-XRCE-DDS-Agent; \
    mkdir build; \
    cd build; \
    cmake ..; \
    make; \
    make install; \
    ldconfig /usr/local/lib/; \
    cd ../..; \
    rm -Rf Micro-XRCE-DDS-Agent;"

# Instalar Gazebo, RQT, RVIZ, y paquetes de ROS2
# libogre1.12.10 required for rviz
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y \
    ~nros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-rviz2 \
    libogre1.12.10 \
    gz-garden \
    && apt autoremove \
    && rm -rf /var/lib/apt/lists/*


# Precompilar todos los paquetes dentro de la carpeta ros2_pkgs
COPY src /home/user/ros2_ws/src

RUN /bin/bash -c "cd /home/user/ros2_ws; \
    apt update; \
    rosdep update; \
    rosdep install -r --from-paths src -i -y --rosdistro ${ROS_DISTRO}; \
    source /opt/ros/${ROS2_DISTRO}/setup.bash; \
    colcon build --symlink-install; \
    rm -rf /var/lib/apt/lists/*;"
    
# Preparar salida del container
COPY docker/px4_ros/ros2/entrypoint.sh /entrypoint.sh
EXPOSE 8888
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]