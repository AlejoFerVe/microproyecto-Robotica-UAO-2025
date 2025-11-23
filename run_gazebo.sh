#!/bin/bash

# Script para lanzar el modelo RC en Gazebo con movimiento en X e Y

echo "================================================"
echo "  Lanzando Modelo RC en Gazebo"
echo "================================================"
echo ""
echo "Características:"
echo "  - Movimiento en eje X (prismatic_x): -5.0 a 5.0 metros"
echo "  - Movimiento en eje Y (prismatic_y): -5.0 a 5.0 metros"
echo ""
echo "Controles:"
echo "  - Usa la GUI de Joint State Publisher para mover el modelo"
echo "  - Arrastra los sliders para cambiar la posición"
echo ""
echo "================================================"
echo ""

# Ir al workspace
cd ~/Escritorio/micro_ws

# Source de ROS2 y workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Lanzar Gazebo
ros2 launch rc_model gazebo_rc.launch.py
