import serial
import paho.mqtt.client as mqtt

#######################################################################################

username = "WjwaZ1GcV475lOxZv3pU"
password = ""
broker = "iot.eie.ucr.ac.cr"
topic = "v1/devices/me/telemetry"
name = "Proyecto_DG"


def on_log(client, userdata, level, buf): #Callback para logging
    print ("log: "+buf )

def on_connect(client , userdata, flags, rc): #Callback cuando se conecta
    if rc == 0:
        print("Connected OK with code", rc)
    else :
        print("Bad connection returned code", rc)

def on_disconnect(client, userdata, flags, rc = 0): #Callback cuando se desconecta
    print( "Disconnected result code " + str(rc))

def on_message(client, userdata , msg): #Callback cuando recibe mensaje
    topic = msg.topic
    m_decode = str(msg.payload.decode( "utf-8", "ignore" ))
    print("message received" ,m_decode)

#######################################################################################


client = mqtt.Client(name)
client.username_pw_set(username)

client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

print("Connecting to broker", broker)
client.connect(broker, 1883, 60)

print("Subscribing to topic", topic)
client.subscribe(topic)

client.loop_start()

serialPort = serial.Serial(port = "/tmp/ttyS1", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

str1 = ""

while True:
    if(serialPort.in_waiting > 0):
        sensor = serialPort.readline()[:-2].decode()
        serialString = serialPort.readline()
        sensor = sensor.split()
        if sensor != []:
            for elemento in range(0,6):
                payload = sensor[elemento * 2].replace(',', '') + sensor[(elemento * 2) + 1]
                str1 = str1 + '"' +sensor[elemento * 2].replace(',', '').replace(':', '') + '"' + ":" + '"' +sensor[(elemento * 2) + 1] + '"'
                if elemento != 5:
                    str1 = str1 + ","
            str1 = "{" + str1 + "}"
            
            ret = client.publish(topic, str1, 1)
            print(str1)
            str1=''
