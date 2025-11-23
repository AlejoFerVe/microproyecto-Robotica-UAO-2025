# Robot de Monitoreo de Calidad de Agua

Sistema robótico autónomo diseñado para el monitoreo de parámetros de calidad de agua (pH y temperatura) en lagos de pesca.

## 📋 Descripción del Proyecto

Este microproyecto de robótica implementa un robot con movilidad en el plano XY capaz de realizar mediciones de pH y temperatura en diferentes puntos de un lago de pesca, permitiendo un monitoreo sistemático de la calidad del agua.

## ✨ Características Principales

- ✅ Movilidad en plano XY mediante articulaciones prismáticas
- ✅ Simulación en Gazebo y RViz
- ✅ Control mediante GUI de ROS2
- ✅ Modelo 3D del robot RC
- ✅ Sistema basado en ROS2 Jazzy
- ✅ Diseñado para medición de pH y temperatura

## 🚀 Instalación

### Requisitos Previos

- ROS2 Jazzy
- Gazebo (GZ Sim)
- RViz2

### Clonar e Instalar

```bash
# Crear workspace (si no existe)
mkdir -p ~/micro_ws/src
cd ~/micro_ws/src

# Clonar repositorio
git clone [URL_DEL_REPOSITORIO] rc_model

# Compilar
cd ~/micro_ws
colcon build --packages-select rc_model

# Source del workspace
source install/setup.bash
```

## 🎮 Uso

### Visualización en RViz (Recomendado)

```bash
ros2 launch rc_model display.launch.py
```

Esto abrirá:
- **RViz**: Visualización 3D del robot
- **Joint State Publisher GUI**: Control manual con sliders

### Simulación en Gazebo

```bash
ros2 launch rc_model gazebo_rc.launch.py
```

## 🎯 Control del Robot

Una vez lanzado RViz o Gazebo, usa la ventana **Joint State Publisher** para controlar el robot:

- **Slider prismatic_x**: Movimiento en eje X (-5.0 a 5.0 metros)
- **Slider prismatic_y**: Movimiento en eje Y (-5.0 a 5.0 metros)

## 📁 Estructura del Proyecto

```
rc_model/
├── urdf/                  # Modelo URDF del robot
│   └── rc_model.urdf
├── meshes/                # Geometrías 3D
│   └── rc_v12.stl
├── launch/                # Archivos de lanzamiento
│   ├── display.launch.py
│   └── gazebo_rc.launch.py
├── config/                # Configuraciones
│   └── display.rviz
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🔧 Características Técnicas

### Articulaciones

- **Tipo**: Prismáticas (deslizantes)
- **Rango X**: -5.0 a 5.0 metros
- **Rango Y**: -5.0 a 5.0 metros
- **Velocidad máxima**: 1.0 m/s
- **Fuerza máxima**: 100.0 N

### Propiedades del Robot

- **Masa total**: 2.1 kg
- **Amortiguamiento**: 10.0
- **Fricción**: 1.0

## 🌊 Aplicación

Este robot fue diseñado para monitoreo de calidad de agua en lagos de pesca, permitiendo:

- 📊 Medición de pH en múltiples puntos
- 🌡️ Registro de temperatura del agua
- 🤖 Desplazamiento autónomo para cobertura del área
- 📈 Recolección de datos para análisis de calidad del agua

## 👥 Autores

| Nombre              | Email/Contacto        |
|---------------------|------------------------|
| Alejandro Fernandez Velasco|  alejandro.fernande_v@uao.edu.co |
| Santiago Aguilar Posada | santiago.aguilar@uao.edu.co|
| Valentina Ramirez Jaramillo | valentina.ramirez_j@uao.edu.co  |

## 🏫 Información Académica

- **Universidad**: Autonoma de Occidente
- **Materia**: Robótica
- **Tipo de proyecto**: Microproyecto
- **Fecha**: Noviembre 2025

## 📜 Licencia

Este proyecto está bajo la Licencia MIT. Ver el archivo `LICENSE` para más detalles.



**Microproyecto de Robótica** | ROS2 Jazzy | 2025
