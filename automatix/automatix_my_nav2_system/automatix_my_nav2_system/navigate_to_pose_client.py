#action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        #creamos el objeto cliente de una accion
        #con parametros
        #nodo
        #tipo de mensaje
        #nombre de la accion
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    #definimos la funcion de mandar goal
    def send_goal(self, position, orientation):
        """
        Manda el goal a Navigate to pose
        Args:
            position:
                x: -5.41667556763
                y: -3.14395284653
                z: 0.0
            orientation:
                x: 0.0
                y: 0.0
                z: 0.785181432231
                w: 0.619265789851
        """


        # crea el mensaje tipo Goal
        # y lo rellena con el argumento dado
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.get_logger().info('Creo objeto')

        #espera a que el servidor este listo
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        

        self.get_logger().info('Navigating to goal: ' + str(goal_pose.pose.position.x) + ' ' +
                      str(goal_pose.pose.position.y) + '...')

        #self._action_client.wait_for_server()
        
        # envia el goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        self.get_logger().info('Se envio el goal')

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    #definimos la funcion de respuesta al goal
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    #definimos la funcion de respuesta al resultado
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    #definimos la funcion de respuesta al feedback
    def feedback_callback(self, feedback_msg):
        """
        Imprimir el feedback del action client
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
        
        future_goal = action_client.send_goal(5,5) 
        rclpy.spin(action_client)
        

    except KeyboardInterrupt:
        action_client.destroy_node()
    finally:
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()
