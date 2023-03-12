#!/bin/bash

export $(grep -v '^#' .env | xargs)

# Obtenemos el usuario actual
uid=$(id -u)
gid=$(id -g)
username=$(id -un)
groupname=$(id -gn)
export uid gid username groupname

echo "Ejecutando contenedor \"${px4_cont_name}\""

docker compose run --rm --name "${px4_cont_name}" px4
