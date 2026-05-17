# Navegación autónoma y evasión reactiva en ROS2 (ROS 2 / Gazebo)

Este repositorio contiene un sistema de guía, navegación y control (GNC) tridimensional y reactivo desarrollado en **ROS 2** (C++) para un vehículo aéreo no tripulado. El sistema permite al dispositivo desplazarse de forma autónoma hacia una secuencia de puntos de destino (*waypoints*) sorteando obstáculos prismáticos complejos mediante una combinación de una Máquina de Estados Finitos (FSM) y un controlador en bucle cerrado.

El proyecto ha sido desarrollado como parte del curso de ROS de la UNED.

## Características Principales

- **Arquitectura Modular Distribuida**: Separación estricta de responsabilidades en tres nodos concurrentes (Nodo base, Nodo controlador y Nodo de seguridad) para garantizar un procesamiento eficiente.
- **Arquitectura Basada en Acciones**: Implementación del protocolo asíncrono Servidor/Cliente de Acciones de ROS 2 (`Maps`) para despachar metas lógicas brindando retroalimentación de distancia restante en tiempo real.
- **Evasión Reactiva de obstáculos (FSM de 5 Estados)**: Algoritmo de evasión estructurado capaz de romper la atracción magnética hacia el objetivo cuando la ruta está obstruida. Gestiona de forma contigua los estados `VOLANDO`, `ROTANDO_ESQUIVA`, `RODEANDO_OBSTACULO`, `ROTANDO_ESQUINA` y `AVANZANDO_TRAS_ESQUINA`.
- **Controlador Proporcional (P-Controller)**: Sistema de control en bucle cerrado que permite al dron "surfear" los muros circundantes calculando dinámicamente velocidades angulares en Z basándose en el error de distancia lateral.
- **Láseres de Precisión (Visión de Túnel)**: Segmentación del LiDAR de 360º en 5 sectores de seguridad combinados con dos sectores de 20º a los laterales del dron (apertura estrecha de 20º a 90º y 270º). Esto elimina el "Síndrome de Abraza-Paredes" y permite al dron escapar de las esquinas en línea recta de forma limpia.
- **Escudo de Coletazo Trasero (Tail Clearance)**: Monitoreo activo de la cola del dron (`dist_atras_`) para exigir un margen de despeje en vacío (>3.0m) antes de reincorporarse al vuelo directo hacia la meta, evitando colisiones cinemáticas al pivotar.
- **Anillo de Seguridad Independiente (Kill Switch)**: Nodo supervisor de supervivencia que barre los 360º del LiDAR de forma autónoma y frena de emergencia si cualquier obstáculo entra en un rango crítico menor a 0.3 metros.
- **Botón de pausa y reanudación del vuelo**: Sistema para detener externamente el vuelo del dron, permitiendo pausar y reaunudar el vuelo en cualquier momento de este.

## Estructura del Paquete `drone_control`

- `src/base_node.cpp`: Nodo central (Action Server). Aloja la FSM de 5 estados, el filtrado de sectores LiDAR, el P-Controller dinámico y publica las TFs dinámicas (`odom` -> `base_link`).
- `src/controller_node.cpp`: Nodo comandante (Action Client). Almacena el plan de vuelo estratégico en 3D (coordenadas X, Y, Z de despegue, navegación y aterrizaje) y las despacha secuencialmente.
- `src/security_node.cpp`: Nodo de control supervisor independiente. Publica en el tópico `emergency_stop` tras analizar colisiones de bajo nivel a menos de 0.3m.
- `launch/drone_launch.py`: Script de orquestación en Python. Inicializa de forma sincronizada el simulador Gazebo Sim con el mapa, procesa el modelo físico en Xacro, levanta el puente de comunicación de tópicos y abre la interfaz gráfica RViz2.
- `urdf/drone.urdf.xacro`: Archivo macro de descripción geométrica, inercial y visual del cuadricóptero, integrando los plugins simulados de telemetría (`OdometryPublisher`), actuación cinemática (`VelocityControl`) y el sensor `gpu_lidar`.
- `worlds/walls.sdf`: Archivo de descripción del entorno simulado en Gazebo que define la topología del circuito de muros y las leyes físicas del entorno.

## 🛠️ Requisitos e Instalación

### Requisitos
- **ROS 2** (Jazzy / Harmonic o distribuciones modernas compatibles)
- **Gazebo Sim** (Ignition / gz_sim)
- Herramientas de compilación: `colcon` y el entorno de construcción `ament_cmake`

### Instalación
1. Clona este repositorio dentro de la carpeta `src` de tu espacio de trabajo de ROS 2 (*workspace*):
   ```bash
   cd ~/proyecto_cursoUNED_ROS2/src
   git clone [https://github.com/pgomez01/Proyecto-Final-ROS2-Curso-UNED.git](https://github.com/pgomez01/Proyecto-Final-ROS2-Curso-UNED.git)
   
2. Compilar el proyecto desde la raíz del workspace:
   ```bash
   cd ~/proyecto_cursoUNED_ROS2/src
   colcon build --packages-select drone_control
3. Cargar el entorno:
   ```bash
   source install/setup.bash

### Ejecución
Para iniciar todos los nodos, Gazebo, RViz2 y la configuración de TFs:
```bash
ros2 launch drone_control drone_launch.py
```
## Galería y Demostraciones
En este apartado se muestra una demostración de un viaje del dron de un punto a otro y como va rodeando los obstáculos a medida que pasan por su camino.
A su vez, se muestran algunas capturas del funcionamiento del LIDAR en RViz2, la arquitectura de Nodos del proyecto a través de la herramienta rqt_graph, el funcionamiento de la parada de emergencia y capturas del dron simulado y el mundo utilizado en Gazebo.

<table>
  <tr>
    <td align="center"><b>Ruta del dron con evasión de obstáculos </b></td>
    <td align="center"><b>Visión Sensorial LiDAR (RViz2)</b></td>
  </tr>
  <tr>
    <td><video src="https://github.com/user-attachments/assets/3025f629-8602-4e69-972f-48d77bd7731a" width="400" controls autoplay loop muted/></td>
    <td><img src="https://github.com/user-attachments/assets/945931d3-c3cf-486b-a620-636cf5e455cd"  width="400"/></td>
  </tr>
  <tr>
    <td align="center"><b>Arquitectura de Nodos (rqt_graph)</b></td>
    <td align="center"><b>Parada de Emergencia Independiente</b></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/f5a0be63-4387-499c-88cd-19f4325f7859"  width="400"/></td>
    <td><img src="https://github.com/user-attachments/assets/931d7cc7-9cb9-4752-8a49-deec37c135af" width="400"/></td>
  </tr>
  <tr>
    <td align="center"><b>Simulación del Dron</b></td>
    <td align="center"><b>Mundo de ROS2</b></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/e3840899-98f2-4e05-8108-31f50c105c37" width="400"/></td>
    <td><img src="https://github.com/user-attachments/assets/c8507be8-593b-4832-8882-255be133e8ee" width="400"/>></td>
  </tr>

</table>

### Autor
- **Pablo Gómez Sánchez**

## Licencia
Este proyecto está licenciado bajo la Licencia Apache 2.0. Consulta l archivo `LICENSE`para más detalle.







   
   
