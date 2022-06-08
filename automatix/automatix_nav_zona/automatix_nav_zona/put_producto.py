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
import yaml


IP_PUERTO = "http://192.168.0.101:8080"

def main():


    idProducto= int(sys.argv[1])
    nombre =sys.argv[2]
    idZona= sys.argv[3]
    cantidad = int(sys.argv[4])
    precio = float(sys.argv[5])

    # 4 subirlo
    headers = {'content-type': 'application/json'}
    data = {
        "idProducto": idProducto, 
        "nombre": nombre,
        "idZona": idZona,
        "cantidad": cantidad,
        "precio": precio

    }
    print(data)
    response = requests.put(IP_PUERTO+'/producto',json=data,headers=headers)
   
    print(response.status_code)
    # 6 enviar al topic un ok
    if(response.status_code == 200):
        a = 1
                

if __name__ == '__main__':
    main()