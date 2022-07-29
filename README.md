# Proyecto_IE-0624_Laboratorio_de_Microcontroladores
El proyecto de sistema de control de tanque de tilapia consiste en automatizar, monitorizar y controlar de forma remota la crı́a de tilapias en estanque para procesos de cultivo intensivo y super intensivo.

Se adjuntan las bibliotecas requeridas.

El proyecto fue desarrollado en ubuntu 20.04 empleando el simulador SimulIDE 1.0.1-RC2, se utilizo también el IDE de Arduino, además de python3.

El archivo tilapia_monitor.ino.mega.hex es el firmware a cargar en el Arduino Mega
El archivo tilapia_monitor.sim.sim1 es el circuito para el simulador SimulIDE.
El archivo to_mqtt.py es el script de python que toma la salida serial y la conecta con el thingsboard.

Instrucciones:
    Se deben abrir los puertos seriales en la terminal con el comando:
            socat PTY,link=/tmp/ttyS0,raw,echo=0 PTY,link=/tmp/ttyS1,raw,echo=0
    
    Ejecutar el script to_mqtt.py en otra terminal ubicado en la carpeta del repositorio
    
    Se debe encender la comunicación serial en el circuito de simulación
    
    Correr la simulación en el simuladorIDE, se debe cargar el firmware del programa .hex al arduino para Ejecutarla simulación.
    
    Los datos seran registrados en la pantalla del simulador, visualizados en el monitor serial
    del Arduino Mega, y mostrados en los paneles del thingsboard.
    
    El sitio de thingsboard con los paneles de visualización de datos se encuntra en el sitio iot.eie.ucr.ac.cr
