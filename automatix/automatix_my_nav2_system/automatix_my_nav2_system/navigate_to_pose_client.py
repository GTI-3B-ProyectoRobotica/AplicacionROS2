"""
    File: navigate_to_pose_client.py
    Author: Rubén Pardo
    Status: DONE
    Date: 14/03/2022
    Description: Este fichero manda un goal con una posición al robot para que vaya a esa posición
    Functions: __init__, _inicializar_goal_pose_desde_parametros, send_goal, goal_response_callback, get_result_callback, feedback_callback, main
"""
#action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateToPoseClient(Node):
    """
    Clase que representa el action client de NavigateToPose que envia goal 
    de posicion y orientacion al action server NavigateToPose a traves de los paramaetros de lanzamiento
    
    attr:
     - _goal_pose(PoseStamped): goal a enviar al action server

    methods:
     - _inicializar_goal_pse_desde_parametros(): Obtiene los parametros de lanzamiento de crea el objeto de clase goal_pose
     - send_goal(): envia el goal al action server NavigateToPose
     - goal_response_callback(): callback que imprime la respuesta del goal recibido del action client
     - get_result_callback(): callback que imprime el resultado del goal recibido del action client
     - feedback_callback(): callback que imprime el feedback del goal enviado recibido del action client

    """
    def __init__(self):
        """
            Inicializa el goal y hace que el robot navegue a ese goal
        """
        super().__init__('navigate_to_pose_client')


        self._inicializar_goal_pose_desde_parametros()
        #creamos el objeto cliente de una accion
        #con parametros
        #nodo
        #tipo de mensaje
        #nombre de la accion
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


    
    def _inicializar_goal_pose_desde_parametros(self):
        """
            funcion para declarar los parametros de lanzamiento del fichero
        """
        self.declare_parameter('pose-x', 0.0)
        self.declare_parameter('pose-y', 0.0)

        self.declare_parameter('orien-x', 0.0)
        self.declare_parameter('orien-y', 0.0)
        self.declare_parameter('orien-z', 0.0)
        self.declare_parameter('orien-w', 1.0)


        self._goal_pose = PoseStamped()

        self._goal_pose.header.frame_id = 'map'
        self._goal_pose.header.stamp = self.get_clock().now().to_msg()

        self._goal_pose.pose.position.x = self.get_parameter('pose-x').get_parameter_value().double_value
        self._goal_pose.pose.position.y = self.get_parameter('pose-y').get_parameter_value().double_value
        self._goal_pose.pose.position.z = 0.0 # siempre sera 0

        self._goal_pose.pose.orientation.x = self.get_parameter('orien-x').get_parameter_value().double_value
        self._goal_pose.pose.orientation.y = self.get_parameter('orien-y').get_parameter_value().double_value
        self._goal_pose.pose.orientation.z = self.get_parameter('orien-z').get_parameter_value().double_value
        self._goal_pose.pose.orientation.w = self.get_parameter('orien-w').get_parameter_value().double_value

        self.get_logger().info('POSE:')
        self.get_logger().info(str(self._goal_pose.pose.position))
        self.get_logger().info('ORIENT')
        self.get_logger().info(str(self._goal_pose.pose.orientation))



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
    rclpy.init(args=args)
    action_client = NavigateToPoseClient()
    
    try:
        
        future_goal = action_client.send_goal() 
        rclpy.spin(action_client)
        

    except KeyboardInterrupt:
        action_client.destroy_node()
    finally:
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()
