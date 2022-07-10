# Los pasos de instalación están basados en la documentación de PX4
# https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html

FROM ubuntu:20.04

###############################################################################
#   Cloning PX4 repo
###############################################################################
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    curl \
    xmlstarlet \
    unzip \
    zip \
    cmake \
    gcc \
    libssl-dev \
    g++ \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive /root/PX4-Autopilot

# Corremos el script pre-armado de setup de PX4.
# El archivo .dockerenv se crea cuando se ejecuta el contenedor, pero no en la
# generación de la imagen, queremos engañar al script para que crea que estamos
# en un docker, e instale las dependencias que necesitamos sin modificarlo directamente.
# Este comando acepta dos parámetros: 
#   --no-sim-tools (no instala Gazebo 11);
#   --no-nuttx (no instala nuttx).
RUN /bin/bash -c "touch /.dockerenv; \
    /root/PX4-Autopilot/Tools/setup/ubuntu.sh; \
    rm /.dockerenv"

###############################################################################
#   Instalación de Fast-DDS y Fast-RTPS
###############################################################################
# Source: https://docs.px4.io/master/en/dev_setup/fast-dds-installation.html 

RUN /bin/bash -c "git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.6.0 /root/FastDDS; \
curl -s "https://get.sdkman.io" | bash; \ 
source /root/.sdkman/bin/sdkman-init.sh; \
sdk install gradle 6.3; \
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 /root/Fast-RTPS-Gen;"

RUN /bin/bash -c "cd /root; \
git clone https://github.com/eProsima/foonathan_memory_vendor.git; \
cd foonathan_memory_vendor; \
mkdir build && cd build; \
cmake ..; \
cmake --build . --target install"

RUN /bin/bash -c "cd /root/FastDDS; \
mkdir build && cd build; \
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..; \
make -j$(nproc --all); \
make install"

RUN export GRADLE_OPTS="-Dfile.encoding=utf-8" && \
/bin/bash -c "PATH=$PATH:/root/.sdkman/candidates/gradle/current/bin; \
cd /root/Fast-RTPS-Gen; \
gradle assemble; \
gradle install"

###############################################################################
#   Limpieza y salida final
###############################################################################
WORKDIR /root/PX4-Autopilot

# Se copian los archivos de inicialización dentro del Docker.
COPY px4-rc.mavlink /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
COPY px4-rc.rtps /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.rtps
COPY px4-rc.simulator /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator
COPY px4-rc.params /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params

# Se compila el código del dron sin correr la simulación
RUN /bin/bash -c "cd /root/PX4-Autopilot; \
DONT_RUN=1 make px4_sitl_rtps gazebo"

# Puerto Rx de RTPS
EXPOSE 2019/udp
# Puerto de Offboard Mavlink
EXPOSE 14580/udp
# Puerto de QGroundControl
EXPOSE 18570/udp

ENV RTPS_PROTOCOL=UDP
ENV RTPS_TX_PORT=2020
ENV RTPS_RX_PORT=2019
ENV QGROUND_IP=127.0.0.1
ENV QGROUND_PORT=14550
ENV ROS2_IP=127.0.0.1

CMD ["/bin/bash"]
