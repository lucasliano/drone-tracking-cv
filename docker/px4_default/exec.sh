#!/bin/bash

export $(grep -v '^#' .env | xargs)

# Preguntamos qué comando se quiere ejecutar
echo "What to do?"
echo "1. Simular drone por defecto (gz_x500)."
echo "2. Simular drone UTN".
echo "3. Ejecutar QGroundControl."
echo "4. Interactive terminal."
echo "q. Exit."

read -p "Select option: " -r -n 1 option
echo ""

case $option in
    1) command="make px4_sitl gz_x500";;

    2) command="make px4_sitl gz_drone_utn";;

    3) {
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

    4) command="/bin/bash";;

    *) {
        echo "Exiting..."
        exit 0
    };;
esac

# Ejecutamos dentro del contenedor el archivo entrypoint y además el comando seleccionado
docker container exec -ti -u "$(id -un)" "${px4_cont_name}" /bin/bash -c "
    source /entrypoint.sh; \
    ${command};"