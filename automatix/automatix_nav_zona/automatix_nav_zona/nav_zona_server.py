# Importar mensajes
from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import IrZona

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node

class Service(Node):

    zonas = {
        'zona1': ['0.0', '0.0'],
        'zona2': ['1.0', '1.0'],
        'zona3': ['2.0', '2.0']
    }

    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('nav_zona_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(IrZona, 'IrZona', self.automatix_nav_zona_callback)

        #declara el objeto publisher pasando como parametros
        # tipo de mensaje
        # nombre del topic
        # tamaño de la cola

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def automatix_nav_zona_callback(self, request, response):
        
        self.get_logger().info(request.zona)
        response.success = True
        # devuelve la respuesta
        return response

def main(args=None):
    # inicializa la comunicacion ROS2
    rclpy.init(args=args)
    # creamos el nodo
    service = Service()
    try:
        #dejamos abierto el servicio
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Cerrando el nodo service Ir a Zona')
    finally:
        #destruimos el nodo
        service.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()

#definimos el ejecutable
if __name__=='__main__':
    main()