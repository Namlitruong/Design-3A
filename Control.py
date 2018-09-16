import paho.mqtt.client as mqtt

Server = "192.168.137.98"
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

	mqttc.publish ("System", Path, 0)
def on_publish (mqttc, obj, mid):
	print ("mid: " + str(mid))

def on_subscribe (mqttc, obj, mid, granted_qos):
	print ("Subscriber: " + str(mid) + "   " + str(granted_qos))

def on_log (mqttc, obj, level, string):
	print (string)
##################################################################
#################--Split the Arrived Message--####################
def splitTheString (inputString):
	global UID
	global DestinationX
	global DestinationY
	global BatteryLv
	tokens = inputString.split(";")
	UID = tokens[0]
	#DestinationX = float (tokens[1])
	#DestinationY = float (tokens[2])
	DestinationX = float (2.5)
	DestinationY = float (2)
	BatteryLv = int (tokens[1])
##################################################################
################-- Path Planning--################################
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
################--Return Path Planning--#########################
def returnPathPlanning (currentX, currentY):
	global returnPath
	#go to corners
	#at points where x > 1
	if (currentX > 1 and ((currentY/0.5)%2) == 0):
		returnPath += "L"
	#at X dropping points
	if (((currentX/0.5)%2) == 0):
		returnPath += "SL"
		currentX += 0.5
	#go to y = 2 axis
	while (currentY < 2):
		returnPath += "S"
		currentY += 0.5
	#go to return path
	returnPath += "SL"
	#go to x = 0 axis
	while (currentX>0):
		returnPath += "S"
		currentX -= 1
	#go to (0,0)
	returnPath += "LS"
	
	

#################################################################
################--UID Data Base--################################
def UIDDataBase (UID):
	switcher = {
	#	"900565168" : "0;0",
                "1205479948": "0,0",
                "1473313548": "1,0",
                "1197842700": "2,0",
                "1732036364": "0,3",
                "1471314444": "1,0.5",
                "1732798732": "2,0.5",
                "1729580556": "1,1",
                "1730039052": "1.5,1",
                "1730031884": "2,1",
                "1731958796": "2.5,1",
                "1463706636": "3,1",
                "1728351756": "1,1.5",
                "1730135820": "2,1.5",
                "1729890316": "1,2",
                "1470158860": "1.5,2",
                "1472705804": "2,2",
                "1466096396": "2.5,2",
                "1461592076": "3,2",
                "1462920972": "3,0",
                "1472246796": "1,3",
                "1733580300": "2,3",
                "1463252492": "3,3",
		"1320283435": "3;1.5",
		"1176890027": "3;0.5",
		"2155339399": "2.5,0",
		"1613815687": "1.5,0"
	}
	return switcher.get (UID, "10;10")
#################################################################
################--UID to Current Position--######################
def UID_to_CurPos (inputData):
	global CurrentX
	global CurrentY
	tokens = inputData.split(";")
	CurrentX = float (tokens[0])
	CurrentY = float (tokens[1])
#################################################################
###################--Main Loop--#################################

mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

mqttc.connect(Server, 1883, 60)
mqttc.subscribe(Topic, 0)

mqttc.loop_forever()
