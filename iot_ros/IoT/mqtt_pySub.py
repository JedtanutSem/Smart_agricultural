import paho.mqtt.client as mqtt
import time

def on_message(client, userdata, message):
    print("received message: " ,str(message.payload.decode("utf-8")))

    

mqttBroker ="localhost"

client = mqtt.Client("python_sub")
client.connect(mqttBroker) 
while 1:
    client.loop_start()

    client.subscribe("/test/mqtt")
    client.on_message=on_message 

    #time.sleep(1)
    client.loop_stop()
