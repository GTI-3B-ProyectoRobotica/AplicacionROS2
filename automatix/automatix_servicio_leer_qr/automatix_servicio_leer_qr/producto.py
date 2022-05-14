"""
    File: zona.py
    Author: Lorelay Pricop
    Status: DONE
    Date: 04/04/2022
    Description: Este fichero contiene la definición para poder representar una Zona
"""
import re
from numpy import double

class Producto:
    """
    Representa un producto 
    
    Attributes:
        _id: id del prducto
        _nombre: Nombre del producto
        _zona: zona al que pertence
        _cantidad: cuantos
        _precio: precio de cada unidad

    Methods:
        get_zona(): devuelve la zona 
        toString(): devuelve los atributos del objeto como un string
    
    """
    def __init__(self, linea):
        """ Inicializa la zona
        
        Args: 
            linea: string con formato idProducto;zona;nombreProducto;cantidad;precio
        """
        # idProducto;zona;nombreProducto;cantidad;precio
        array = linea.split(";")
        self._zona = array[1]
        # prducto
        self._id = array[0]
        self._nombre = array[2]
        self._cantidad = array[3]
        self._precio = array[4]


    def get_zona(self):
        """Devuelve el nombre de la zona
        Return: 
            nombre: el nombre de la zona"""
        return self._zona
    
    def get_punto_medio(self):
        """Devuelve el punto medio de la zona
        Return: 
            punto_medio: el punto medio de la zona"""
        return self._punto_medio

    
    def toString(self):
        """
            return:
                (str) idProducto;zona;nombreProducto;cantidad;precio
        """
        return self._id+";"+self._zona +";"+self._nombre+  ";" + self._cantidad +  ";" + self._precio

    def toStringArgsLineaComando(self):
        """
            String formateado para pasarlo por linea de comando
            return:
                (str) idproducto nombre idzona cantidad precio
        """
        return self._id+" "+self._nombre +" "+self._zona+  " " + self._cantidad +  " " + self._precio
    
