"""
    File: post_mapa.py
    Author: Juan Ferrera y Rubén Pardo
    Status: DONE
    Date: 27/03/2022
    Description: Este fichero obtiene un pgm, lo pasa a png, luego a base64 y lo sube a la base de datos
"""
from cmath import log
import sys
import os
from PIL import Image
import base64
import requests
import json
from geometry_msgs.msg import Twist
import yaml


IP_PUERTO = "http://192.168.85.84:8080"

def main(args=None):

    imgBase64 = ''
    # 1 Obtener fichero pgm
    path = '../../automatix_my_nav2_system/config/'
    #path = 'automatix_my_nav2_system/config/'
    for file in os.listdir(path):
        filename, extension  = os.path.splitext(file)
        
        if extension == ".pgm":
            # 2 Transformarlo a png
            new_file = "{}.png".format(filename)

            # 3 guardarlo
            with Image.open(path+file) as im:
                im.save(new_file)

            # Transoformarlo a base 64
            imgBase64 = base64.b64encode(open(new_file,"rb").read())

            # borrar el fichero
            os.remove(new_file)
        # if
    # for

    resolution_yaml = ""
    with open(path+"my_map.yaml", 'r') as f:
        data = yaml.load(f)
        resolution_yaml = data['resolution']

    imgStr = imgBase64.decode('utf-8')
    # 4 subirlo
    headers = {'content-type': 'application/json'}
    data = {
        "idMapa": 1, 
        "imagen": imgStr,
        "resolucion": resolution_yaml

    }
    response = requests.post(IP_PUERTO+'/mapa',json=data,headers=headers)
   

    # 6 enivat al topic un ok
    if(response.status_code == 200):
        a = 1
                

if __name__ == '__main__':
    main()