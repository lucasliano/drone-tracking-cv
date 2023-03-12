FROM px4io/px4-dev-ros2-foxy:latest

# Instalar Gazebo Garden
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt update && apt install -y \
        gz-garden \
    && rm -rf /var/lib/apt/lists/*

# Preparar salida del container
COPY docker/px4_ros/px4/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "/bin/bash" ]