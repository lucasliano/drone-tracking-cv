#!/bin/bash

export $(grep -v '^#' .env | xargs)

# Obtenemos el usuario actual
uid=$(id -u)
gid=$(id -g)
username=$(id -un)
groupname=$(id -gn)
export uid gid username groupname

docker compose up
docker compose down