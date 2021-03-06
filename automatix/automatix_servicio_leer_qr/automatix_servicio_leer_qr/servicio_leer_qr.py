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
from automatix_custom_interface.srv import IrZona

#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy,QoSProfile


from automatix_servicio_leer_qr.producto import Producto
from automatix_servicio_leer_qr.predict import Predict

import cv2
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
        self._caja_rota = False
        self._bridge = CvBridge()
        # suscribirse al topic image
        self._suscribirse_image()

        # crear un cliente a nav zona
        #crea el objeto cliente
        self.client_nav_zona = self.create_client(IrZona, 'IrZona')
        #cada segundo revisa si el servicio esta activo
        while not self.client_nav_zona.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('el servicio nav zona no esta activo, prueba de nuevo...')
        

        self.nombreFoto = "caja"
        self.idFoto = 0
        



    def _suscribirse_image(self):
        """
            Funcion para suscribirse al topic image_raw
        """
        self.get_logger().info("ME SUSCRIBO A IMAGE_RAW")
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',#'/camera/image_raw' en simulacion, image en real
            self._image_callback,
            QoSProfile(depth = 10,reliability = ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscription  
        self.get_logger().info("SUSCRITO")

    def guardar_imagen(self, msg):
        """
            Funcion para guardar las iimagenes del robot
            pulsando la q o dsecartarlas con otra tecla
            Usado para obtener el dataset
        """
        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Imagen capturada por el robot", cv_image)
            nombre = self.nombreFoto + self.idFoto
            self.idFoto += 1
            cv2.imwrite("/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/dataset"+nombre+".jpg", cv_image)
            cv2.waitKey(0)
            self.get_logger().info("guardado")  
        except CvBridgeError as e:
            self.get_logger().info(e) 

    def _image_callback(self, msg):
        """
            Callback de la suscripcion del topic camera/image_raw
            args:
                msg:
                    width: double
                    heigth: double
                    data: [][]

        """
        self.get_logger().info(str(self._leer_qr))
        if self._leer_qr:
            # leer la imagen
            self.get_logger().info("LEER")
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # guardar imagen
            cv2.imwrite("/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_servicio_leer_qr/automatix_servicio_leer_qr/fotoDesdeRos.jpg", cv_image)
            # predecir 
            a = Predict()
            str_prediccion = a.predecir()
            print("=========================================")
            print("Resultado de predict:"+str_prediccion)
            
            if str_prediccion == "caja":
                # robot centrado, leer qr
                try:

                    self.get_logger().info("leer qr")
                    #cv2.imshow("Imagen",cv_image)
                    #cv2.waitKey(0)
                    #cv2.destroyAllWindows()
                    cv_image = cv2.imread("/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_servicio_leer_qr/automatix_servicio_leer_qr/qrReal.jpeg")
                    det = cv2.QRCodeDetector()
                    val, pts, st_code=det.detectAndDecode(cv_image)
                    ##val id;zona;nombre;cantidad;precio
                    #self._publicar_nav_zona("zona1")
                    self.get_logger().info("LEO: " + str(val))
                    if self._isValidFormatoTextoQr(val):
                        self.get_logger().info("VAL: "+val)
                        
                        self._leer_qr = False
                        
                        producto = Producto(val)

                        self._publicar_nav_zona(producto.get_zona(),producto)
                except CvBridgeError as e:
                    self.get_logger().info(e)
            # if--------------
            elif str_prediccion == "caja_rota":
                self.get_logger().info("centrado, caja rota")
                self._publicar_nav_zona("cajas_rotas",producto)
            # elif--------------
        # if------------------
           


    def _isValidFormatoTextoQr(self,text):
        """
            args:
                text:String
            rerturn T/F si tiene el formato correcto: id;zona;nombre;cantidad;precio
        """
        if text !="":    
            return True
        return False
        

    def _publicar_nav_zona(self,zona, producto):
        """
            Metodo que publica en el topic /IrZona
            args:
                zona: nombre de la zona a ir
                producto: producto que transporta a esa zona

        """
        
        req = IrZona.Request()
        req.zona = zona
        req.producto = producto.toString()
        self.get_logger().info("publico en "+str(req))
        self.future = self.client_nav_zona.call_async(req)

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
        self.get_logger().info(request.activar)
        if request.activar == "True":
            self.get_logger().info("activar")
            # ACTIVAR LECTURA QR 
            self._leer_qr = True
            response.success = True
        elif not request.activar  == "True":
            # DESACTIVAR LECTURA QR
            self._leer_qr = False
            response.success = True
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False

        # devuelve la respuesta
        return response


    def _mover_robot(self, direccion):
        """
            mover el robot hacia esa direccion
        """
        self._centrando_caja = False



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