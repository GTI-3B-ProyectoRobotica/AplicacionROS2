# Importar mensajes
from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import IrZona

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node
import sys
sys.path.insert(1, '../automatix_nav_zona')
from automatix_nav_zona.zona import Zona

class Service(Node):

    _zonas = {}

    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('nav_zona_server')
        #cambiarlo por ruta relativa
        self._rellenarDiccionarioZonas("/home/lorelay/turtlebot3_ws/src/AplicacionROS2/automatix/zonas/zonas.txt")
        
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(IrZona, 'IrZona', self.automatix_nav_zona_callback)


    def automatix_nav_zona_callback(self, request, response):
        #comprobamos que request.zona (lo que mandamos) existe en el diccionario
        if request.zona in self._zonas:
            self.get_logger().info(self._zonas[request.zona].toString())
            response.success = True
        else:
            response.success = False
        # devuelve la respuesta
        return response
    
    def _rellenarDiccionarioZonas(self, file):
        """Metodo para rellenar el diccionario con zonas
        Args:
            file: nombre del fichero donde están guardadas las zonas    
        """
        arrayZonas = Zona.zonaFromTxt(file)
        for zona in arrayZonas:
            self._zonas[zona.get_name()] = zona


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