"""
    File: automatix_guardar_zona_server.py
    Author: Pablo Enguix
    Status: DONE
    Date: 04/04/2022
    Description: Este fichero guarda las zonas que se le mandan desde un servidor en un fichero zonas.txt
"""

# Importar mensajes
from multiprocessing.connection import Client
from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import GuardarZona

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node

class Service(Node):
    def __init__(self):
        """
            Crea el servicio para guardar zonas  
        """
        #constructor con el nombre del nodo
        super().__init__('automatix_guardar_zona_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(GuardarZona, 'automatix_guardar_zona', self.my_first_service_callback)

    def my_first_service_callback(self, request, response):
        """
            Guarda las zonas validadas en un fichero zonas.txt
        """
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        if request.zonas != "":
            zonas = request.zonas.split(";")
            zonasValidas = ""
            self.get_logger().info("%s"%zonas)

            zonasValidas = self.validar_zonas(zonas)
            if(zonasValidas == ""):
                self.get_logger().info('Error al guardar zonas')
                response.success = False
                return response
                    
            self.get_logger().info('Zonas validadas. ')
            try:
                f = open("/home/pablo/turtlebot3_ws/src/AplicacionROS2/automatix/zonas/zonas.txt", "w")
                f.write(zonasValidas)
                f.close()
                self.get_logger().info("Se está guardando %s" %zonasValidas)
            except Exception as exc:
                # estado de la respuesta
                # si no se ha dado ningun caso anterior
                self.get_logger().error(exc)
                response.success = False
            response.success = True
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False

        # devuelve la respuesta
        return response
    def validar_zonas(self, zonas):
        zonasValidas = ""
        for zona in zonas:
            if(len(zona.split(":")) == 2 and len(zona.split(",")) == 4 and zona[-1:] != ";"):
                zonasValidas += zona + ";"
            else:
                return ""
        return zonasValidas

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