#!/bin/bash

# Asegurarse que los submódulos están en el repositorio
git submodule update --init --recursive

# Agregar una referencia al remoto de PX4 original (necesario para compilar).
cd PX4-Autopilot
git remote add upstream https://github.com/PX4/PX4-Autopilot
git fetch upstream

# Salir del modo "detached HEAD"
git checkout main
