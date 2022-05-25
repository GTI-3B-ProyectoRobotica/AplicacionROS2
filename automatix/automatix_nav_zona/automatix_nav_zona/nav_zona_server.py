"""
    File: nav_zona_server.py
    Author: Lorelay Pricop
    Status: DONE
    Date: 04/04/2022
    Description: Este fichero obtiene una lista de zonas, crea un diccionario de zonas y, a partir de una zona, calcula su punto medio
                 y hace que el robot se mueva a ese punto
"""
# Importar mensajes

from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import IrZona
from automatix_custom_interface.srv import ActivarLeerQr

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node
import sys
sys.path.insert(1, '../automatix_nav_zona')


from automatix_nav_zona.zona import Zona
from automatix_nav_zona.producto import Producto
import os


class Service(Node):
    """
        Servicio que controla el ir a una zona. Escucha mensajes de IrZona{zona:"nombre de la zona"}, este nombre se comprueba
        en una array en local de las zonas que conoce (Se inicializa al principio leyendo un fichero donde se van actualizanco)
        Si la zona es transportista cuando el goal de nav_client termina con exito public en el servicio de 
        leer qr un activar = true. Por otra parte, le llegara una zona y un producto y descativara la lectura de qr publicando al 
        servicio un activar = false. Cuando se resuelve con extio el goal del nav client llamara al script put_product para actualizar la 
        base de datos

        Tambien escucha mensajes de tipe IrZona = zonaAdded para actualizar sus zonas guardadas leyendo otra vez del fichero

        atributos:
            _zonas: las zonas que tiene guardada
            _path_to_zonas_fila: direccion al fichero donde se guardan las zonas del mapa
            _producto: producto que puede enviarse a una zona
            _isTransportista: booleano que controla si el robot esta yendo a una zona del transportista o una zona de paquetes

        metodos:
            automatix_nav_zona_callback: callback del servicio
            _rellenarDiccionarioZonas: funcion que lee del fichero y actualiza las zonas
            send_goal: funcion que envia los goal al cliente de navegacion
            goal_response_callback: callback del goal que envia si se acepto o no
            get_result_callback: callback que se llama cuando el goal termino
            feedback_callback
            publicarEnServicioLeerQr: funcion que publica en el servicio dde leer qr
            put_producto: funcion que llama al fichero put_producto.py
    
    """
    _zonas = {}
    _path_to_zonas_file = "/home/ruben/turtlebot3_ws/src/AplicacionROS2/automatix/zonas/zonas.txt"

    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('nav_zona_server')
        #cambiarlo por ruta relativa
        self._rellenarDiccionarioZonas(self._path_to_zonas_file)
        self._isTransportista = False
        self.client_leer_qr = self.create_client(ActivarLeerQr, 'servicio_leer_qr')
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(IrZona, 'IrZona', self.automatix_nav_zona_callback)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


    def automatix_nav_zona_callback(self, request, response):

        self._goal_pose = PoseStamped()
        #comprobamos que request.zona (lo que mandamos) existe en el diccionario
        self.get_logger().info('LLEGA: '+str(request.zona))
        if request.zona in self._zonas:
            self._isTransportista = str(request.zona) == "transportista"
            self.get_logger().info('EXISTE ZONA: '+str(request.zona))

            if not self._isTransportista:
                # si no es transportista se envia el producto que lleva el robot a la zona
                self.get_logger().info('Me llego el prodcuto: '+request.producto)
                self._producto = Producto(request.producto)
                self.get_logger().info('Me llego el prodcuto: '+str(self._producto.toString()))

            
            x,y = self._zonas[request.zona].get_punto_medio()
            self.get_logger().info('ME MUEVO A: x:'+str(x)+' y:'+str(y))
            
            self._goal_pose.header.frame_id = 'map'
            self._goal_pose.header.stamp = self.get_clock().now().to_msg()
            self._goal_pose.pose.position.x = x
            self._goal_pose.pose.position.y = y
            self._goal_pose.pose.position.z = 0.0 # siempre sera 0

            self._goal_pose.pose.orientation.x = 0.0
            self._goal_pose.pose.orientation.y = 0.0
            self._goal_pose.pose.orientation.z = 0.0
            self._goal_pose.pose.orientation.w = 0.0

            self.get_logger().info('POSE:')
            self.get_logger().info(str(self._goal_pose.pose.position))
            self.get_logger().info('ORIENT')
            self.get_logger().info(str(self._goal_pose.pose.orientation))
            self.send_goal()

            response.success = True
        elif request.zona == "zonas_added":
            self.get_logger().info('ZONA ADDED')
            self._rellenarDiccionarioZonas(self._path_to_zonas_file)
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
    
    def send_goal(self):
        """
         Funcion que envia el goal al action server Navigate To pose
        """

        # crea el mensaje tipo Goal
        # y lo rellena con el argumento dado

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._goal_pose
        self.get_logger().info('Creo objeto')

        #espera a que el servidor este listo
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        

        self.get_logger().info('Navigating to goal: ' + str(self._goal_pose.pose.position.x) + ' ' +
                      str(self._goal_pose.pose.position.y) + '...')

        
        # envia el goal
        self.send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        self.get_logger().info('Se envio el goal')

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
            Funcion que recibe la respuesta del goal
            arg:
                future: objeto que contiene el resultado 
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_logger().info("empieza es transortista:")
        self.get_logger().info(str(self._isTransportista))
        if(not self._isTransportista):
            # ha reconocido el qr y se dirige a una zona, desactivar el qr
            self.publicarEnServicioLeerQr(False)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    
    def get_result_callback(self, future):
        """
            Funcion de respuesta al resultado
            args:
                future: objeto que contiene el resultado de la respuesta
        """
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        self.get_logger().info("termino es transortista:")
        self.get_logger().info(str(self._isTransportista))
        if(self._isTransportista):
            #ha llegado a la zona transportista, activar el qr
            self.publicarEnServicioLeerQr(True)
        else:
            self.put_producto(self._producto)
        #rclpy.shutdown()

    #definimos la funcion de respuesta al feedback
    def feedback_callback(self, feedback_msg):
        """
        Funcion que recibe el feedbak del action client y lo imprime en pantalla
        args:
            feedback_msg: 
                goal_id=unique_identifier_msgs.msg.UUID, 
                feedback=nav2_msgs.action.NavigateToPose_Feedback(
                    current_pose=geometry_msgs.msg.PoseStamped(
                    header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=403, nanosec=649000000), frame_id='map'), 
                    pose=geometry_msgs.msg.Pose(
                        position=geometry_msgs.msg.Point(x=, y=, z=), 
                        orientation=geometry_msgs.msg.Quaternion(x=, y=, z=, w=))), 
                        navigation_time=builtin_interfaces.msg.Duration(sec=8, nanosec=96000000), 
                        number_of_recoveries=0, 
                        distance_remaining=0.23206885159015656))
        """
        # TODO mirar si el punto esta dentro de un radio del punto enviado
        feedback = feedback_msg.feedback
       
    def publicarEnServicioLeerQr(self, activar):
        """
            Metodo que le publica un mensaje en el servicio /servicio_leer_qr
            args:
                activar: T/F
        """
        self.get_logger().info("publico en qr: "+str(activar))
        req = ActivarLeerQr.Request()
        req.activar = str(activar)
        self.future = self.client_leer_qr.call_async(req)

    def put_producto(self,producto):
        self.get_logger().info('LLAMO A PUT PRODUCTO')
        os.system("pip install requests")
        os.system("python /home/ruben/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_nav_zona/automatix_nav_zona/put_producto.py "+producto.toStringArgsLineaComando())
      

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