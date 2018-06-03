import paho.mqtt.client as mqtt

Server = "192.168.137.210"
Topic = "UID"

def on_connect (mqttc, obj, flags, rc):
	print("rc:" +str(rc))

def on_message (mqtt, obj, msg):
	print(msg.topic + ""+str(msg.qos)+""+str(msg.payload))

def on_publish (mqttc, obj, mid):
	print("mid: " + str(mid))

def on_subscribe (mqttc, obj, mid, granted_qos):
	print("Subcribed: " + str(mid) + " " + str(granted_qos))

def on_log (mqttc, obj, level, string):
	print(string)


mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

mqttc.connect(Server, 1883,60)
mqttc.subscribe(Topic, 0)

mqttc.loop_forever() 
