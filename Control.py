import paho.mqtt.client as mqtt

Server = "192.168.137.98" # Serve IP
Topic = "Robot1" 

def on_connect (mqttc, obj, flags, rc):
	print ("rc: " + str(rc))

def on_message (mqtt, obj, msg):
	print (msg.topic + "   " + str (msg.qos) + "   " + str(msg.payload))
	ArrivedMessage = str(msg.payload)
	splitTheString (ArrivedMessage)
	UID_to_CurPos (UIDDataBase(UID))
	PathPlanning (CurrentX, CurrentY, DestinationX, DestinationY)
	
	print ("Path:		 " + str(Path))
	print ("UID:  		 " + str(UID))
	print ("CurrentX:	 " + str(CurrentX))
	print ("CurrentY:	 " + str(CurrentY))
	print ("DestinationX:	 " + str(DestinationX))
	print ("DestinationY:	 " + str(DestinationY))
	print ("BatteryLv:	 " + str(BatteryLv))

	mqttc.publish ("System", Path, 0) #After process the position, the path will be send back to the Robot via "System" topic
def on_publish (mqttc, obj, mid):
	print ("mid: " + str(mid))

def on_subscribe (mqttc, obj, mid, granted_qos):
	print ("Subscriber: " + str(mid) + "   " + str(granted_qos))

def on_log (mqttc, obj, level, string):
	print (string)
##################################################################
#################--Split the Arrived Message--####################
# In here, I still asume that Distination location is from the Arduino for testing. However, it can be assign by the UI module
def splitTheString (inputString):
	global UID
	global DestinationX
	global DestinationY
	global BatteryLv
	tokens = inputString.split(";")
	UID = tokens[0]
	#DestinationX = float (tokens[1])
	#DestinationY = float (tokens[2])
	DestinationX = float (2) # X coordinator of the destination  
	DestinationY = float (1.5)# Y Coordinator of the destination 
	BatteryLv = int (tokens[3])
##################################################################
################-- Path Planning--################################
# By input the Current and Destination position, the Path can be calculated and send back to Arduino
def PathPlanning (CurrentX, CurrentY, DestinationX, DestinationY):
	global Path
	Path = ""
	if (DestinationX % 1 == 0.5):
		print ("debug1")
		while (CurrentX  < DestinationX + 0.5):
			if (CurrentX >=  1):
				CurrentX += 0.5
			else: 
				CurrentX += 1
			Path += "S"
		Path += "L"
		while (CurrentY < DestinationY ):
			if (CurrentY < 3):
				CurrentY += 0.5
			else:
				CurrentY += 1
			Path += "S"
		print ("x" + str(CurrentX))
		print ("y" + str(CurrentY))
		if (CurrentX > DestinationX):
			Path += "LS"
	else:
		print ("debug2")
		while (CurrentX + 0.5 <= DestinationX):
			if (CurrentX >= 1):
				CurrentX += 0.5
			else:
				CurrentX += 1
			Path += "S"
		Path += "L"
		while (CurrentY + 0.5 <= DestinationY):
			if (CurrentY < 3):
				CurrentY += 0.5
			else:
				CurrentY += 1
			Path += "S"
#		if (CurrentX < DestinationX):
#			Path = Path + "RS"
#		if (CurrentY < DestinationY):
#			Path = Path + "LS"
#################################################################
################--UID Data Base--################################
#Put the code of UID database in here 

#################################################################
################--UID to Current Position--######################
#From UID database we know the Current location and convert it into float for calculation
def UID_to_CurPos (inputData):
	global CurrentX
	global CurrentY
	tokens = inputData.split(";")
	CurrentX = float (tokens[0])
	CurrentY = float (tokens[1])
#################################################################
###################--Main Loop--#################################
# This part declare and initialize the MQTT connection
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

mqttc.connect(Server, 1883, 60) # Establish connection 
mqttc.subscribe(Topic, 0) # Subscribe the robot publish data 

mqttc.loop_forever()
