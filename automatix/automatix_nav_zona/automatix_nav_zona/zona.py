import re

from numpy import double

"""
Este fichero contiene la definición para poder representar una Zona
Classes: 
    Zona

Author: Lorelay Pricop
Version: 04/04/2022
"""

class Zona:
    """
    Representa una zona definida por el usuario. Las coordenadas están guardadas en un txt
    
    Attributes:
        _name: Nombre de la zona
        _xi: x inferior
        _yi: y inferior
        _xs: x superior
        _ys: y superior
        _punto_medio: punto medio de la zona

    Methods:
        _calcular_punto_medio(): calcula el punto medio de dos coordenadas
        get_name(): devuelve el nombre de la zona
        toString(): devuelve los atributos del objeto como un string
        zonaFromTxt(file): devuelve las zonas formateadas desde un txt
    
    """
    def __init__(self, linea):
        """ Inicializa la zona
        
        Args: 
            linea: nombre y coordenadas fomateadas como "zona1:xi$yi$xs$ys"
        """
        array1 = linea.split(':')
        name = array1[0]
        coordenadas = array1[1].split('$')
        self._nombre=name
        self._xi=coordenadas[0]
        self._yi=coordenadas[1]
        self._xs=coordenadas[2]
        self._ys=coordenadas[3]
        self._punto_medio = self._calcular_punto_medio()


    def get_name(self):
        """Devuelve el nombre de la zona
        Return: 
            nombre: el nombre de la zona"""
        return self._nombre
    
    def get_punto_medio(self):
        """Devuelve el punto medio de la zona
        Return: 
            punto_medio: el punto medio de la zona"""
        return self._punto_medio

    
    def toString(self):
        return self._nombre+": inferior(" +self._xi +  ", " + self._yi +  "), superior(" + self._xs +  ", " + self._ys +  "), " + "Punto medio: " + str(self._punto_medio) 
    
    def _calcular_punto_medio(self):
        """ Calcula el punto medio de la zona
        Return:
            x, y: las coordenadas x e y del punto medio
        """

        x = (double(self._xi)+double(self._xs))/2
        y = (double(self._yi)+double(self._ys))/2

        return x,y


    @staticmethod
    def zonaFromTxt(file):
        """Lee de un txt las zonas y las convierte en un array

        Args:
            file: el nombre del fichero

        Return:
            arrayZonas: array de linea por zona, separados por ; en el txt
        """
        lines = ""
        arrayZonas = []
        with open(file) as f:
            lines = f.read()

        arrayZonasString = lines.split(';') 
        
        for zonaString in arrayZonasString:
            if len(zonaString)>2:
                arrayZonas.append(Zona(zonaString))

        return arrayZonas
