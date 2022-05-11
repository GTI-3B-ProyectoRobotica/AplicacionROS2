"""
    File: out_producto.py
    Author: Lorelay Florescu
    Status: 
    Date: 05/05/2022
    Description: 
"""
from cmath import log
import sys
import os
import requests
import json
from geometry_msgs.msg import Twist
import yaml


IP_PUERTO = "http://10.236.57.230:8080"

def main():


    idProducto= sys.argv[1]
    idZona= sys.argv[2]
    # 4 subirlo
    headers = {'content-type': 'application/json'}
    data = {
        "idProducto": idProducto, 
        "idZona": idZona

    }
    response = requests.put(IP_PUERTO+'/producto',json=data,headers=headers)
   
    print(response.status_code)
    # 6 enviar al topic un ok
    if(response.status_code == 200):
        a = 1
                

if __name__ == '__main__':
    main()