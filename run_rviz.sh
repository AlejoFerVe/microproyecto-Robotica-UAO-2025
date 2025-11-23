#!/bin/bash

# Script para visualizar el modelo RC en RViz

echo "================================================"
echo "  Visualizando Modelo RC en RViz"
echo "================================================"
echo ""
echo "Características:"
echo "  - Visualización 3D del modelo RC"
echo "  - Control manual de joints con sliders"
echo "  - Sin física (solo visualización)"
echo ""
echo "================================================"
echo ""

# Ir al workspace
cd ~/Escritorio/micro_ws

# Source de ROS2 y workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Lanzar RViz
ros2 launch rc_model display.launch.py
