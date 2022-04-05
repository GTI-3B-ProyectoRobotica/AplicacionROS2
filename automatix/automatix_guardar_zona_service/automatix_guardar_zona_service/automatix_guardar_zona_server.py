# Importar mensajes
from multiprocessing.connection import Client
from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import GuardarZona

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node

class Service(Node):
    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('automatix_guardar_zona_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(GuardarZona, 'automatix_guardar_zona', self.my_first_service_callback)

    def my_first_service_callback(self, request, response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        if request.zonas != "":
            zonas = request.zonas.split(";")
            zonasValidas = ""
            self.get_logger().info("%s"%zonas)
            for zona in zonas:
                if(len(zona.split(":")) == 2 and len(zona.split(",")) == 4):
                    zonasValidas += zona + ";"
                elif(zona !=""):
                    self.get_logger().info('Error al guardar zonas: zona %s' %zona)
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

#class Service(Node):
#    mis_zonas = MisZonas
#    def __init__(self):
#        #constructor con el nombre del nodo
#        super().__init__('automatix_guardar_zona_server') 
#        # declara el objeto servicio pasando como parametros
#        # tipo de mensaje
#        # nombre del servicio
#        # callback del servicio
#        self.srv = self.create_service(GuardarZona, 'automatix_guardar_zona', self.my_first_service_callback)
#
#        self.declare_parameter("zonas", "inicio:0,0,0,0;")
#        self.mis_zonas.zonas = self.get_parameter("zonas").get_parameter_value().string_value
#        self.create_timer(2, self.timer_callback)
#    def timer_callback(self):
#        # cada timer_period segundos cargamos los parámetros en el mensaje
#        # y lo imprimimos por pantalla
#        self.mis_zonas.zonas = self.get_parameter("zonas").get_parameter_value().string_value
#        self.get_logger().info('Mis zonas son: %s' %self.mis_zonas.zonas)
#        self.get_logger().info("Mi primera zona es: %s" %self.mis_zonas.zonas.split(";")[0])
#        self.mis_zonas.zonas = self.mis_zonas.zonas.split(";")
#        zonasValidas = ""
#        for zona in self.mis_zonas.zonas:
#            if(len(zona.split(":")) == 2 and len(zona.split(",")) == 4):
#                zonasValidas += zona + ";"
#        self.mis_zonas.zonas = zonasValidas
#        self.my_first_service_callback()
#    def my_first_service_callback(self):
#        try:
#            f = open("/home/pablo/turtlebot3_ws/src/AplicacionROS2/automatix/zonas/zonas.txt", "w")
#            f.write(self.mis_zonas.zonas)
#            f.close()
#            self.get_logger().info("Se está guardando %s" %self.mis_zonas.zonas)
#        except Exception as exc:
#            # estado de la respuesta
#            # si no se ha dado ningun caso anterior
#            self.get_logger().error(exc)
#
#
#def main(args=None):
#    # inicializa la comunicacion ROS2
#    rclpy.init(args=args)
#    # creamos el nodo
#    service = Service()
#    try:
#        #dejamos abierto el servicio
#        rclpy.spin(service)
#    except KeyboardInterrupt:
#        service.get_logger().info('Cerrando el nodo service')
#    finally:
#        #destruimos el nodo
#        service.destroy_node()
#        #cerramos la comunicacion
#        rclpy.shutdown()
#
##definimos el ejecutable
#if __name__=='__main__':
#    main()