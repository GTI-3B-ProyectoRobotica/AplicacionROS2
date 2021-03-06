"""
    File: escaneo_autonomo_server.py
    Author: Rubén Pardo
    Status: DONE
    Date: 30/03/2022
    Description: Este fichero contiene un servicio el cual cuando se arranca mueve el robot por el mapa y detecta colisiones para obtener un pgm del mapa
    Functions: __init__, _suscribirse_scan, listener_scan_callback, is_colision, my_first_service_callback, mover_robot, _terminar_escaneo, main
"""

# Importar mensajes
from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import EscanearMsg
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random
from threading import Timer
#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node
import os
import subprocess

class Service(Node):
    def __init__(self):
        """
        Crea el servicio de escanear el mapa, controla la direccion, comprueba colisiones y establece un temporizador de 60 segundos
        """
        #constructor con el nombre del nodo
        super().__init__('escaneo_autonomo_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.get_logger().info('INICIO SERVER')
        self.srv = self.create_service(EscanearMsg, 'escaneo_autonomo', self.my_first_service_callback)

        #flag para controlar cuando esta escaneando o no
        self._is_escaneando = False

        # contador para controlar el cambio de direccion para evitar que se atasque en esquinas
        self._cont_cambiar_direccion = 20
        self._cont_cambiar_direccion_temp = self._cont_cambiar_direccion

        # grio del robot
        self._velocidad_angular = 1.5078

        # suscribirse a scan para comprobar las colisiones
        self._suscribirse_scan()

        # definir un timer para que el escaneo no sea infinito
        self.timer = Timer(60,self._terminar_escaneo,args=[]) # tiempo en segundos
        
        #declara el objeto publisher pasando como parametros
        # tipo de mensaje
        # nombre del topic
        # tamaño de la cola

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def _suscribirse_scan(self):
        """
            Se suscribe al topic /scan para comprobar las colisiones
        """
        self.subscriber= self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 
        # prevent unused variable warning
        self.subscriber   

    def listener_scan_callback(self,msg):
        """
            Callback de la suscripcion al topic /scan
            args:
                msg: Mensaje del topic scan, ranges[] array de 360 grados con las distancias a las colisiones
            
        """
        self.get_logger().info(str(msg.ranges[0]))
        # coger los 40 grados de delante
        distancias_pared_delante = msg.ranges[0:15]
        distancias_pared_delante.extend(msg.ranges[345:359])

        if self.is_colision(distancias_pared_delante) and self._is_escaneando:
            # a las 20 colisiones cambiamos de direccion
            self._cont_cambiar_direccion_temp-=1
            if(self._cont_cambiar_direccion_temp == 0):
                self._cont_cambiar_direccion_temp = self._cont_cambiar_direccion
                self._velocidad_angular = -self._velocidad_angular
            self.mover_robot(0.0,self._velocidad_angular)
        elif self._is_escaneando:
            # mover
            #self.mover_robot(0.1,0.0)  # en real
            self.mover_robot(0.3,0.0)  # en simulacion
          
    def is_colision(self, distancias):
        """
            Metodo que calcula si hay una colision cerca con n distancias
            args:
                distancias: array de distancias a la pared de delante 

            return: True si una distancia es menor a 0.4
        """
        colision = False
        #self.get_logger().info('Colisiones ==============')
        for d in distancias:
        #    self.get_logger().info('Distancia: ' +str(d))
            if d <= 0.4:
                colision = True
        #        self.get_logger().info('Colision===============================')
                break
        return colision

    def my_first_service_callback(self, request, response):
        """
            Comprueba si debe escanear o no
        Args:
            request : Lo que recibe
            response : Lo que devuelve

        Returns: response
        """
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        if request.escanear == "escanear":
            self.get_logger().info('Recibí escanear')
            self.timer.start()
            self.mover_robot(0.3,0.0)  # en simulacion
            #self.mover_robot(0.1,0.0) En real
            self._is_escaneando = True
            response.success = True
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False

        # devuelve la respuesta
        return response
    
    def mover_robot(self,vLineal,vAngular):
        """
            Metodo que cambia la velocidad angular y lineal del robot publicando al topic cmd_vel
            args:
                vLineal: velocidad lineal, float
                vAngular: velocidad angular float
        """
        # crea un mensaje tipo Twist para mover el robot luego
        msg = Twist()
        msg.linear.x = vLineal
        msg.angular.z = vAngular
        # publica el mensaje
        self.publisher.publish(msg)

    def _terminar_escaneo(self):
        """
            Metodo que se llama al terminar el timer, para el robot
            compila el mapa, lanza post_mapa y cierra el nodo
        """
        self.get_logger().info('TERMINA TIMER ==============')
        # parar robot
        self._is_escaneando = False
        self.mover_robot(0.0,0.0)
       
        self.get_logger().info("compilo mapa")
        os.system("ros2 run nav2_map_server map_saver_cli -f $HOME/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_my_nav2_system/config/my_map")
        self.get_logger().info("install requests")
        os.system("pip install requests")
        self.get_logger().info("lanzo post mapa")
        os.system("python /home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_escaneo_autonomo/automatix_escaneo_autonomo/post_mapa.py")
      

def main(args=None):
    # inicializa la comunicacion ROS2
    rclpy.init(args=args)
    # creamos el nodo
    service = Service()
    try:
        #dejamos abierto el servicio
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Cerrando el nodo service')
    finally:
        #destruimos el nodo
        service.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()

#definimos el ejecutable
if __name__=='__main__':
    main()