# Importar mensajes
from multiprocessing.connection import Client
from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import BorrarZona

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node

class Service(Node):
    def __init__(self):
        """
            Crea el servicio para borrar una zona  
        """
        #constructor con el nombre del nodo
        super().__init__('automatix_borrar_zona_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(BorrarZona, 'automatix_borrar_zona', self.my_first_service_callback)

    def my_first_service_callback(self, request, response):
        """
            Borra la zona del fichero zonas.txt
        """
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        if request.zona != "":
            try:
                f = open("/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/zonas/zonas.txt", "r+")
                zonas = f.readline()
                zonas = zonas.split(";")
                zonas = zonas[:-1]
                self.get_logger().info("zonas: %s" %zonas)
                zonaS = ""
                for zona in zonas:
                    if not str(zona).find(request.zona):
                        self.get_logger().info("zona que se borra: %s" %zona)
                        zonas.remove(zona)
                
                for zona in zonas:
                    zonaS += str(zona) + ";"
                self.get_logger().info("zonas que se guardan: %s" %zonaS)
                f.seek(0)
                f.write(zonaS)
                f.truncate()
                f.close()
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