#! /usr/bin/env python
import sys
sys.path.insert(0, "/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_predict_yolo/automatix_predict_yolo/")
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
import cv2

# We do this because of a bug in kinetic when importing cv2

#import subprocess
#new_proc = subprocess.Popen(["rosversion", "-d"], stdout=subprocess.PIPE)
#version_str = new_proc.communicate()[0]
#ros_version = version_str.decode('utf8').split("\n")[0]
ros_version = "kinetic"
if ros_version == "kinetic":
    try:
        sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    except Exception as ex:
        print(ex)
        print("Its already removed..../opt/ros/kinetic/lib/python2.7/dist-packages")


# DEFINE_string('classes', '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_predict_yolo/automatix_predict_yolo/new_names.names', 'path to classes file')
# DEFINE_string('weights', '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_predict_yolo/automatix_predict_yolo/ckeckpoints/yolov3_train_134.tf',
#                    'path to weights file')
# DEFINE_boolean('tiny', False, 'yolov3 or yolov3-tiny')
# DEFINE_integer('size', 416, 'resize images to')
# DEFINE_string('image', '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_predict_yolo/automatix_predict_yolo/caja1.jpeg', 'path to input image')
# DEFINE_string('tfrecord', None, 'tfrecord instead of image')
# DEFINE_string('output', '/home/tostyfis/output.png', 'path to output image')
# DEFINE_integer('num_classes', 1, 'number of classes in the model')


def predecir(args=None):
    print("DESDE predecir: ")
    print(args)
    classes2 = '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_predict_yolo/automatix_predict_yolo/new_names.names'
    weights = '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_predict_yolo/automatix_predict_yolo/ckeckpoints/yolov3_train_4.tf'
    tiny = False
    size = 416
    image = '/home/tostyfis/turtlebot3_ws/src/AplicacionROS2/automatix/automatix_predict_yolo/automatix_predict_yolo/fotoDesdeRos.jpg'
    tfrecord = None
    output = '/home/tostyfis/output.png'
    num_classes = 1

    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    if len(physical_devices) > 0:
        tf.config.experimental.set_memory_growth(physical_devices[0], True)

    yolo = YoloV3(classes = num_classes)

    yolo.load_weights( weights).expect_partial()
    logging.info('weights loaded')

    class_names = [c.strip() for c in open( classes2).readlines()]
    logging.info('classes loaded')

    if  tfrecord:
        dataset = load_tfrecord_dataset(
             tfrecord,  classes2,  size)
        dataset = dataset.shuffle(512)
        img_raw, _label = next(iter(dataset.take(1)))
    else:
        img_raw = tf.image.decode_image(
            open( image, 'rb').read(), channels=3)

    img = tf.expand_dims(img_raw, 0)
    img = transform_images(img,  size)

    t1 = time.time()
    boxes, scores, classes, nums = yolo(img)
    t2 = time.time()
    logging.info('time: {}'.format(t2 - t1))

    logging.info('detections:')

    # class_names[int(classes[0][i])] el nombre de la clase que ha detectado
    # np.array(scores[0][i] la accuracy 
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




def main(args=None):
    a = 1
if __name__ == '__main__':
    print("xDDDDDDDDDDDDDDDDDDDDDDDDDd")
    main()

