# AplicacionROS2
Codigo fuente del robot ros2 del proyecto de robotica
La idea principal del proyecto es programar un robot turtle3 para que sea capaz de clasificar cajas por tipo en distintas
zonas de un espacio escaneado con lidar

# Instalacion del entorno en ubuntu 20.04
1. Instalar ROS2 
```bash
#Autorizar la clave GPG de ROS: 
sudo apt update &&  sudo apt install curl gnupg2 lsb-release 

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg


# Añadir el repositorio:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. Instalar el Foxy y también el compilador colcon:
```bash
sudo apt update 
sudo apt install ros-foxy-ros-base
sudo apt-get install python3-colcon-common-extensions
```

3. Configurar el entorno
```bash
source /opt/ros/foxy/setup.bash
```

4. Instalar gazebo, cartographer, navigation y turtlebot3
```bash
#gazebo 11
sudo apt-get install ros-foxy-gazebo-*

#cartographer 
sudo apt install ros-foxy-cartographer ros-foxy-cartographer-ros

#navigation 2
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup

#turtlebot 3
source ~/.bashrc
sudo apt install ros-foxy-dynamixel-sdk ros-foxy-turtlebot3-msgs ros-foxy-turtlebot3

#configuración del entorno
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

Para facilitar las cosas al usuario y que no tenga que configurar la variable ROS_DOMAIN cada vez que se conecte al sistema (lo configurado en ~/.bashrc se pierde cuando se restaure una máquina virtual) se crea un fichero /usr/local/bin/foxy para que el usuario ejecute simplemente source foxy con el siguiente contenido:
```bash
source /opt/ros/foxy/setup.sh
export ROS_DOMAIN_ID=30 #TURTLEBOT3
source /usr/share/gazebo/setup.sh
```

Y le damos permiso de ejecución para todos:
```bash
sudo chmod +x /usr/local/bin/foxy
```
