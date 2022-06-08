import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import cv2

import time
from absl import app, flags, logging
from absl.flags import FLAGS
import numpy as np
import tensorflow as tf
from .models import (
    YoloV3, YoloV3Tiny
)
from .dataset import transform_images, load_tfrecord_dataset
from .utils import draw_outputs

class Predict:

    def __init__(self):
        a =1

    def predecir(args=None):

        print("DESDE predecir: ")

        classes_path = '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_servicio_leer_qr/automatix_servicio_leer_qr/new_names.names' #path to classes file
        weights = '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_servicio_leer_qr/automatix_servicio_leer_qr/ckeckpoints/yolov3_train_4.tf' #path to weights file
        tiny = False #yolov3 or yolov3-tiny
        size = 416 #resize images to
        
        # Descomentar cuando si tome fotos de cajas (en real)
        #image = '/home/ruben/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_servicio_leer_qr/automatix_servicio_leer_qr/fotoDesdeRos.jpg' #path to input image
        
        image = '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_servicio_leer_qr/automatix_servicio_leer_qr/caja2.jpeg'

        tfrecord = None #tfrecord instead of image
        output = '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_servicio_leer_qr/automatix_servicio_leer_qr/output.png' #path to output image
        num_classes = 1 #number of classes in the model

        physical_devices = tf.config.experimental.list_physical_devices('GPU')
        
        if len(physical_devices) > 0:
            tf.config.experimental.set_memory_growth(physical_devices[0], True)

        if tiny:
            yolo = YoloV3Tiny(classes=num_classes)
        else:
            yolo = YoloV3(classes=num_classes)

        yolo.load_weights(weights).expect_partial()
        logging.info('weights loaded')

        class_names = [c.strip() for c in open(classes_path).readlines()]
        logging.info('classes loaded')

        if tfrecord:
            dataset = load_tfrecord_dataset(
                tfrecord, classes_path, size)
            dataset = dataset.shuffle(512)
            img_raw, _label = next(iter(dataset.take(1)))
        else:
            img_raw = tf.image.decode_image(
                open(image, 'rb').read(), channels=3)

        img = tf.expand_dims(img_raw, 0)
        img = transform_images(img, size)

        t1 = time.time()
        boxes, scores, classes, nums = yolo(img)
        t2 = time.time()
        logging.info('time: {}'.format(t2 - t1))

        logging.info('detections:')
        clase = ""
        for i in range(nums[0]):
            clase = class_names[int(classes[0][i])]
            logging.info('\t{}, {}, {}'.format(class_names[int(classes[0][i])],
                                            np.array(scores[0][i]),
                                            np.array(boxes[0][i])))

        img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
        img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
        cv2.imwrite(output, img)
        logging.info('output saved to: {}'.format(output))

        return clase
    
    

#def main(args=None):
#    a=1
#
#
#
#if __name__ == '__main__':
#    main()
