"""
    File: servicio_leer_qr.py
    Author: Ruben Pardo
    Date: 07/06/2022
    Description: Servicio que se suscribe al topic /image para leer las imagenes para leer qr. 
                Recibe ordenes para activarse o desactivarse 
"""

# Importar mensajes
from multiprocessing.connection import Client
from sensor_msgs.msg import Image
from automatix_custom_interface.srv import ActivarLeerQr

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy,QoSProfile

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class Service(Node):
    """
        clase Service
            atributos:
                _leer_qr (bool) indica si tratar o no las imagenes que llegan del topic image
    
    """

    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('automatix_servicio_leer_qr') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(ActivarLeerQr, 'servicio_leer_qr', self.service_callback)
        self._leer_qr = False
        self._bridge = CvBridge()
        # suscribirse al topic image
        self._suscribirse_image()

    def _suscribirse_image(self):
        """
            Funcion para suscribirse al topic image_raw
        """
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            QoSProfile(depth = 10,reliability = ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscription  


    def _image_callback(self, msg):
        """
            Callback de la suscripcion del topic camera/image_raw
            args:
                msg:
                    width: double
                    heigth: double
                    data: [][]

        """ 
        if self._leer_qr:
            
            try:
                cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                cv2.imshow("Imagen",cv_image)

                cv2.waitKey(0)
                cv2.destroyAllWindows()
                
            except CvBridgeError as e:
                self.get_logger().info(e)
            



    def service_callback(self, request, response):
        """
            Funcion que recibe las ordenes de activar la lectura de qr o desactivarla
            args:
                request:
                    activar True|False

            returns:
                response:
                    success True|False
        """

        if request.activar:
            # ACTIVAR LECTURA QR 
            self._leer_qr = True
            response.success = True
        elif not request.activar:
            # DESACTIVAR LECTURA QR
            self._leer_qr = False
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