# Intercomunicación entre PX4 y ROS2

Simulación de [Gazebo Garden](https://gazebosim.org/home) con drones usando el firmware [PX4](https://docs.px4.io/main/en/), interconectado con una computadora de abordo corriendo [ROS2 Humble](https://docs.ros.org/en/humble/index.html).

## Utilización

Para ejecutar solo PX4:

```bash
# In one terminal
cd docker/px4_default
./run.sh

# Other terminal
cd docker/px4_default
./exec.sh
```

Para ejecutar PX4 junto con ROS2:

```bash
# In one terminal
cd docker/px4_ros
./run.sh

# In different terminals
cd docker/px4_ros
./exec.sh
```

## Crear paquetes de ROS2 propios

Todos los paquetes de ROS2 se deben incluir dentro de la carpeta `src`. Al realizar cambios, seleccionar la opción de compilar en el script `./exec.sh`.

## Agregar un modelo personalizado a PX4

Los pasos hechos para agregar un modelo propio en la simulación fueron los siguientes:

1. Definir un archivo airframe en la carpeta `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes` de nombre: `XXXXX_<SIMULATOR>_<MODEL_NAME>`, donde los números son aleatorios. En este archivo, se pueden cargar todos los parámetros específicos del modelo, así como comandos de inicialización adicionales.

2. Agregar en el archivo `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` el nombre del archivo usado para definir el airframe en el paso anterior.

3. Agregar dentro de la carpeta `PX4-Autopilot/Tools/simulation/gz/models` el modelo a simular.

![Drone UTN en Gazebo](media/gz_drone.png)

Fuentes:

* [Adding new frame to PX4](https://docs.px4.io/main/en/dev_airframes/adding_a_new_frame.html)

* [Gazebo Simulation on PX4](https://docs.px4.io/main/en/sim_gazebo_gz/).

* Repositorio con el [modelo del drone](https://github.com/utnpiddrones/drone_utn_model).

## Árbol de directorios de PX4

En esta sección se explican los archivos principales en el directorio del [repositorio de PX4](https://github.com/PX4/PX4-Autopilot).

```bash
PX4-Autopilot/
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
|   ├── simulation/
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

  * **init.d-posix/**: Contiene el script de inicialización `rcS`. Todo drone, físico o simulado, ejecuta estas instrucciones.

* **Tools/**: Contiene scripts de ayuda para la ejecución.

  * **simulation**:  Contiene modelos de drones y mundos de Gazebo a ser ejecutados.

* **src/**: Contiene todos los comandos ejecutables sobre la consola de PX4, definidos como clases de C++.

* **launch/**: Contiene los archivos "roslaunch" para ejecutar simulaciones con múltiples drones. Estos archivos funcionaban con ROS1 y MAVROS, por lo que no se garantiza que funcionen actualmente.

## Inicialización del dron

Al iniciar el drone, este pone a disposición una [consola](https://docs.px4.io/master/en/concept/system_startup.html) para comunicarse, y ejecuta el script de inicialización `ROMFS/px4fmu_common/init.d-posix/rcS`, el cual setea todos los [parámetros de control](https://docs.px4.io/master/en/advanced_config/parameter_reference.html) y canales de comunicación de [MAVLink](https://mavlink.io/en/), inicia la simulación, etc. Los comandos utilizados dentro de estos scripts son módulos de PX4, que se encuentran en la carpeta `src/`, definidos como clases de C++.

Los distintos archivos dentro de la carpeta `init.d-posix` se ocupan de:

* **px4-rc.mavlink**: Establecer las comunicaciones de Mavlink (con IPs y números de puerto).

* **px4-rc.params**: Setear los parámetros del drone.

* **px4-rc.rtps**: Establecer los canales de comunicación de RTPS.

* **px4-rc.simulator**: Preparar el entorno de simulación.

## Arquitectura

El siguiente es un diagrama de la [arquitectura de comunicaciones de PX4](https://docs.px4.io/master/en/concept/px4_systems_architecture.html). Sobre la placa PixHawk se corre PX4, el sistema de control. Sobre la computadora a bordo, conectada a través de [MAVLink](https://mavlink.io/en/) o tópicos de ROS2, se ejecutan todas las tareas no esenciales, como la comunicación con la cámara y la GCS, la detección de objetos y prevención de colisiones, etc.

![Diagrama en bloques de la comunicación](https://docs.px4.io/master/assets/img/px4_arch_fc_companion.c430665d.svg)

Un esquema más específico de la [comunicación interna del stack de vuelo de PX4](https://docs.px4.io/main/en/concept/architecture.html) se muestra en la siguiente imagen. El protoclo de mensajes [uORB](https://docs.px4.io/master/en/msg_docs/) es aquel que maneja XRCE-DDS.

![Diagrama avanzado de comunicación](https://docs.px4.io/master/assets/img/PX4_Architecture.fa89af6b.svg)

### Protocolos de comunicación: MAVLink y XRCE-DDS

Aún cuando el protocolo [MAVLink](https://mavlink.io/en/) puede ser usado para la totalidad de las comunicaciones, el protocolo [XRCE-DDS](https://docs.px4.io/main/en/middleware/xrce_dds.html), es el más adecuado para "librerías de robótica para computer vision y otros usos donde la información en tiempo real hacia/desde los actuadores y sensores es esencial para el control del vehículo".

XRCE-DDS se conforma por un `Client`, ejecutado sobre PX4, y un `Agent`, ejecutado sobre la computadora a bordo. Entre ellos, se encargan de enviar y recibir los distintos mensajes uORB (o de ROS2) a través de UART o UDP de manera serializada.

![Diagrama de conexión con PX4 mediante XRCE-DDS](https://docs.px4.io/main/assets/img/architecture_xrce-dds_ros2.fed61809.svg)

MAVLink, por otro lado, se maneja en parte de manera automática por PX4, para la comunicación con [QGC](http://qgroundcontrol.com/), y se usa por parte del usuario para offboard control (una computadora aparte). De manera independiente, [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) es una librería de python que permite manejar la recepción y transmisión de mensajes bajo este protocolo. Integrado con ROS2 viene el paquete de Mavros.

### Definición de puertos

Estos son los puertos que se usan durante la simulación de Gazebo, definidos en esta página.

![Puertos](https://docs.px4.io/master/assets/img/px4_sitl_overview.d5d197f2.svg)

## Decisiones tomadas en base a conflictos pasados

El paquete binario `ros-humble-ros-gz-bridge` sirve para la versión de ROS2 Humble, y la versión de Gazebo Fortress, según el [repositorio de GitHub](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge). Por lo tanto, se compilan desde el código fuente.

## Links útiles

* Integración entre ROS2 y Gazebo Garden: [ros_gz_bridge](https://gazebosim.org/docs/garden/ros2_integration). [Repositorio de GitHub](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge).

* Formato [SDF para los sensores](http://sdformat.org/spec?elem=sensor).

* Contenedores de [Docker de PX4](https://docs.px4.io/main/en/test_and_ci/docker.html).

* [PX4 y ROS2 User Guide](https://docs.px4.io/main/en/ros/ros2_comm.html)

## Siglas

* **GCS**: Ground Control Station.
* **XRCE**: eXtremely Resource Constrained Environments.
* **DDS**: Data Distribution Service.
* **ROS2**: Robot Operating System 2.
* **MAVLink**: Micro Air Vehicle Link.
* **QGC**: QGroundControl.
