# PX4

Simulación de [Gazebo](http://classic.gazebosim.org/) con [PX4](https://docs.px4.io/master/en/). Interconectado con [RTPS](https://docs.px4.io/master/en/middleware/micrortps.html) a través de [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html). Basado, en la imagen de Docker [utnpiddrones/px4](https://hub.docker.com/repository/docker/utnpiddrones/px4).

# Uso

La imagen acepta las siguientes variables de entorno:

* `RTPS_PROTOCOL`: Protocolo de comunicación para RTPS acepta [ "UDP" | "UART" ]. (Valor por defecto: "UDP"). 
* `ENV RTPS_TX_PORT`: Puerto de transmisión para RTPS. (Valor por defecto: "2020").
* `RTPS_RX_PORT`: Puerto de lectura para RTPS. (Valor por defecto: "2019").
* `ENV QGROUND_IP`: Dirección IP de la QGC. (Valor por defecto: "127.0.0.1").
* `ENV QGROUND_PORT`: Puerto de QGC. (Valor por defecto: "14550").
* `ROS2_IP`: Dirección IP donde se encuentran los nodos de ROS2. (Valor por defecto: "127.0.0.1").

Presenta por defecto la interfaz gráfica del simulador Gazebo. Para activar la interfaz gráfica, ejecutar (una sola vez entre reinicios de la computadora):

```sh
$ xhost local:root
```

## Modelo de servicio Docker Compose

```yaml
services:      
  px4:
    image: "utnpiddrones/px4:latest"

    environment:
      # Necesarias para el uso de la GUI.
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1

    # Se definen las variables de ambiente en un archivo llamado ".env".
    env_file: .env

    volumes:
      # Necesario para la interfaz gráfica
      - "/tmp/.X11-unix:/tmp/.X11-unix:rw"

    # Necesario para la interfaz gráfica, da acceso a los periféricos.
    privileged: true

    # Terminal interactiva
    tty: true

    # Nombre del contenedor al ejecutar.
    container_name: "px4_cont"

    # Comando a ser ejecutado.
    command: /bin/bash -c "make px4_sitl_rtps gazebo"
```


# Árbol de directorios de PX4
Dentro del Docker [utnpiddrones/px4](https://hub.docker.com/repository/docker/utnpiddrones/px4), se clona el [repositorio de GitHub de PX4](https://github.com/PX4/PX4-Autopilot). El árbol de directorios con los contenidos más relevantes del mismo es como sigue:
```sh
# Directorio por defecto: /root/PX4-Autopilot

/root/
└── PX4-Autopilot/
    ├── ROMFS/
    |   └──  px4fmu_common/
    |       ├── airframes/
    |       └── init.d-posix/
    |           ├── px4-rc.mavlink
    |           ├── px4-rc.params  
    |           ├── px4-rc.rtps
    |           ├── px4-rc.simulator
    |           └── rcS
    ├── Tools/
    |   ├── sitl_gazebo/
    |   |       ├── models/
    |   |       ├── worlds/
    |   |       └── scripts/
    |   |           └── jinja_gen.py
    |   ├── setup/
    |   |   └── ubuntu.sh
    |   ├── sitl_run.sh
    |   └── sitl_multiple_run.sh
    |
    ├── src/
    └── launch/
```

* **~/ROMFS/px4fmu_common/**: Contiene todos los comandos de la consola de PX4 para inicializar un drone.
    
    * **airframes/**: Contiene los scripts de inicialización específicos para los modelos de drone.

    * **init.d-posix/**: Contiene el script de inicialización `rcS`. Todo drone, físico o simulado, ejecuta esta instrucción.

* **Tools/**: Contiene scripts de ayuda para la ejecución.

    * **sitl_gazebo**: Submódulo [sitl_gazebo](https://github.com/PX4/PX4-SITL_gazebo/tree/5610c3fb441a2f3babc8ad7a63c8c4ce3e40abfa). Contiene modelos de drones y mundos de Gazebo por defecto.

        * **models/**: modelos por defecto, definidos de forma paramétrica con archivos jinja.sdf.

        * **worlds/**: mundos por defecto.

        * **scripts/jinja_gen.py**: Compila los modelos. Cada modelo es definido de manera paramétrica, debiendo definir al momento de iniciar la simulación los puertos y las IPs para comunicarse.

    * **setup/ubuntu.sh**: Script de instalación, usado para la construcción de la imagen de Docker.

    * **sitl_run.sh**: Ejecuta una simulación de Gazebo y spawnea un drone. Para ello, se le deben pasar los siguientes parámetros:
    ```sh
    ./Tools/sitl_run.sh /root/PX4-Autopilot/build/px4_sitl_rtps/bin/px4 none gazebo iris none /root/PX4-Autopilot /root/PX4-Autopilot/build/px4_sitl_rtps
    ```

    * **sitl_multiple_run.sh**: TODO

* **src/**: Contiene todos los comandos ejecutables sobre la consola de PX4, definidos como clases de C++.

* **launch/**: Contiene los archivos "roslaunch" para ejecutar simulaciones con múltiples drones. Estos archivs funcionaban con ROS1 y MAVROS, por lo que no se garantiza que funcionen actualmente. TODO




# Inicialización del dron
Al iniciar el drone, este pone a disposición una [consola](https://docs.px4.io/master/en/concept/system_startup.html) para comunicarse, y ejecuta el script de inicialización `ROMFS/px4fmu_common/init.d-posix/rcS`, el cual setea todos los [parámetros de control](https://docs.px4.io/master/en/advanced_config/parameter_reference.html), canales de comunicación de [MAVLink](https://mavlink.io/en/) y [RTPS]((https://docs.px4.io/master/en/middleware/micrortps.html)), inicia la simulación, etc. Los comandos utilizados dentro de estos scripts son módulos de PX4, que se encuentran en la carpeta `src/`, definidos como clases de C++.

Los distintos archivos dentro de la carpeta `init.d-posix` se ocupan de:

* **px4-rc.mavlink**: Establecer las comunicaciones de Mavlink (con IPs y números de puerto).

* **px4-rc.params**: Setear los parámetros del drone.

* **px4-rc.rtps**: Establecer los canales de comunicación de RTPS.

* **px4-rc.simulator**: Preparar el entorno de simulación.


## Agregando un modelo personalizado a PX4
Se puede [definir un modelo propio](https://docs.px4.io/master/en/dev_airframes/adding_a_new_frame.html#config-file) siguiendo los siguientes pasos:

1. Definir un archivo airframe en la carpeta `ROMFS/px4fmu_common/init.d-posix/airframes` de nombre: `XXXXX_<MODEL_NAME>`. En este, se pueden cargar todos los parámetros específicos del modelo, así como comandos de inicialización adicionales.

2. TODO 

3. TODO

## Scripts automáticos de inicialización

TODO


# Arquitectura

El siguiente es un diagrama de la [arquitectura de comunicaciones de PX4](https://docs.px4.io/master/en/concept/px4_systems_architecture.html). Sobre la placa PixHawk se corre PX4, el sistema de control. Sobre la computadora a bordo, conectada a través de [MAVLink](https://mavlink.io/en/) o RTPS, se ejecutan todas las tareas no esenciales, como la comunicación con la cámara y la GCS, la detección de objetos y prevención de colisiones, etc.

<img src="https://docs.px4.io/master/assets/img/px4_arch_fc_companion.c430665d.svg" alt="Diagrama en bloques de la comunicación" >

Un esquema más específico de la [comunicación interna del stack de vuelo de PX4](https://docs.px4.io/master/en/concept/architecture.html) se muestra en la siguiente imagen. El protoclo de mensajes [uORB](https://docs.px4.io/master/en/msg_docs/) es aquel que maneja RTPS. 

<img src="https://docs.px4.io/master/assets/img/PX4_Architecture.fa89af6b.svg" alt="Diagrama avanzado de comunicación"/>

## Protocolos de comunicación: MAVLink y RTPS

Aún cuando el protocolo [MAVLink](https://mavlink.io/en/) puede ser usado para la totalidad de las comunicaciones, el protocolo [RTPS](https://docs.px4.io/master/en/middleware/micrortps.html), es el más adecuado para "librerías de robótica para computer vision y otros usos donde la información en tiempo real hacia/desde los actuadores y sensores es esencial para el control del vehículo".

RTPS se conforma por un `microRTPS Client`, ejecutado sobre PX4, y un `microRTPS Agent`, ejecutado sobre la computadora a bordo. Entre ellos, se encargan de enviar y recibir los distintos mensajes uORB (o de ROS2) a través de UART o UDP de manera serializada.

<img src="https://docs.px4.io/master/assets/img/architecture.94eff761.png" alt="Diagrama de conexión con PX4 mediante RTPS">

Ejecutado en la consola de PX4, en el archivo `px4-rc.rtps`.
```
> micrortps_client start|stop|status [options]
  -t <transport>          [UART|UDP] Default UART
  -d <device>             UART device. Default /dev/ttyACM0
  -l <loops>              How many iterations will this program have. -1 for infinite. Default -1.
  -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms
  -b <baudrate>           UART device baudrate. Default 460800
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms
  -r <reception port>     UDP port for receiving. Default 2019
  -s <sending port>       UDP port for sending. Default 2020
  -i <ip_address>         Select IP address (remote) values: <x.x.x.x>. Default: 127.0.0.1
```

Ejecutado sobre la computadora a bordo como un proceso daemon.
```
$ ./micrortps_agent [options]
  -t <transport>          [UART|UDP] Default UART.
  -d <device>             UART device. Default /dev/ttyACM0.
  -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms.
  -b <baudrate>           UART device baudrate. Default 460800.
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms.
  -r <reception port>     UDP port for receiving. Default 2019.
  -s <sending port>       UDP port for sending. Default 2020.
  -n <set namespace>      Set a namespace for the micrortps_agent.
```
MAVLink, por otro lado, se maneja en parte de manera automática por PX4, para la comunicación con [QGC](http://qgroundcontrol.com/), y se usa por parte del usuario para offboard control (una computadora aparte). De manera independiente, [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) es una librería de python que permite manejar la recepción y transmisión de mensajes bajo este protocolo. Integrado con ROS2 viene el paquete de Mavros.


## Definición de puertos

<image src="https://docs.px4.io/master/assets/img/px4_sitl_overview.d5d197f2.svg"></image>


# Siglas

* **GCS**: Ground Control Station.
* **RTPS**: Real Time Publish Subscribe Protocol.
* **DDS**: Data Distribution Service.
* **ROS2**: Robot Operating System 2.
* **MAVLink**: Micro Air Vehicle Link.
* **QGC**: QGroundControl.