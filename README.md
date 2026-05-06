# Sistema de Control de Movimiento de un Dron con Evasión de Obstáculos

Este proyecto consiste en el desarrollo de un sistema de control de vuelo en **ROS2** (C++) que permite a un dron ejecutar maniobras de navegación hacia puntos de destino mediante servidores de acción, integrando un sistema reactivo de evasión de obstáculos basado en datos de sensores LIDAR.

El proyecto ha sido desarrollado como parte del curso de ROS de la UNED.

## Características Principales

- **Arquitectura Basada en Acciones**: Implementación de un Servidor de Acción para manejar trayectorias de vuelo y proporcionar retroalimentación en tiempo real.
- **Evasión de Obstáculos**: Lógica reactiva que analiza lecturas LIDAR en el cono frontal para detectar distancias críticas y ejecutar maniobras de seguridad.
- **Gestión de Transformaciones (TFs)**: Uso de TFs dinámicas para la posición del dron (`odom` -> `base_link`) y TFs estáticas para el montaje del sensor (`base_link` -> `lidar_link`).
- **Simulación**: Validación inicial en **RViz2** y simulación final en **Gazebo**.

## Estructura del Paquete

- `base_node`: Nodo servidor de la acción que calcula el movimiento y publica las TFs dinámicas.
- `controlador_node`: Cliente de la acción que envía los puntos de destino.
- `seguridad_node`: Procesa los datos del LIDAR y dicta las maniobras de evasión.

## Requisitos e Instalación

### Requisitos
- ROS2 Jazzy (o distribución compatible)
- Colcon y herramientas de compilación de ROS2

### Instalación
1. Clonar el repositorio en el `src` de tu workspace:
   ```bash
   cd ~/drone_project_ws/src
   git clone https://github.com/pgomez01/sistema-control-dron-ros2.git
   ```
2. Compilar el proyecto desde la raíz del workspace:
   ```bash
   cd ~/drone_project_ws
   colcon build --packages-select drone_control
   ```
3. Cargar el entorno:
   ```bash
   source install/setup.bash
   ```

## Ejecución
Para iniciar todos los nodos y la configuración de TFs:
```bash
ros2 launch drone_control drone_launch.py
```

## Autor
- **Pablo Gómez Sánchez**

## Licencia
Este proyecto está licenciado bajo la Licencia Apache 2.0. Consulta el archivo `LICENSE` para más detalles.
