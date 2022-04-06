# Importar mensajes
from geometry_msgs.msg import Twist
from automatix_custom_interface.srv import IrZona

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

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
        self._rellenarDiccionarioZonas("/home/ruben/turtlebot3_ws/src/AplicacionROS2/automatix/zonas/zonas.txt")
        
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(IrZona, 'IrZona', self.automatix_nav_zona_callback)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


    def automatix_nav_zona_callback(self, request, response):

        self._goal_pose = PoseStamped()
        #comprobamos que request.zona (lo que mandamos) existe en el diccionario
        if request.zona in self._zonas:
            x,y = self._zonas[request.zona].get_punto_medio()
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
        rclpy.shutdown()

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
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


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