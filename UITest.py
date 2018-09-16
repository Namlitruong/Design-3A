import serial # import Serial Library
import numpy as np  # Import numpy
import matplotlib.pyplot as plt #import matplotlib library
from drawnow import *
import warnings
import matplotlib.cbook
from matplotlib.widgets import Button
from mpl_toolkits.axes_grid1.inset_locator import InsetPosition
from matplotlib.font_manager import FontProperties
from matplotlib.offsetbox import AnnotationBbox, OffsetImage
import paho.mqtt.client as mqtt


Server = "192.168.137.98"
Topic = "Robot1"

warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)

###################Draw Buttons to Send Final Destination####################
## global uid
Path = ""
returnPath = ""
def on_connect (mqttc, obj, flags, rc):
	print ("rc: " + str(rc))

def on_message (mqtt, obj, msg):
	print (msg.topic + "   " + str (msg.qos) + "   " + str(msg.payload))
	ArrivedMessage = str(msg.payload)
	splitTheString (ArrivedMessage)
	UID_to_CurPos (UIDDataBase(UID))

	print ("UID:  		 " + str(UID))
	print ("CurrentX:	 " + str(CurrentX))
	print ("CurrentY:	 " + str(CurrentY))
	print ("BatteryLv:	 " + str(BatteryLv))

	#mqttc.publish ("System", Path, 0)
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
	global BatteryLv
	tokens = inputString.split(";")
	UID = tokens[0]
	BatteryLv = int (tokens[1])

#################################################################
################--Return Path Planning--#########################
def returnPathPlanning (returnPath, currentX, currentY):
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
	returnPath += "LSL"
	print(returnPath)

#################################################################
################--UID Data Base--################################
def UIDDataBase (UID):
	switcher = {
                "1205479948": "00;00",
                "1473313548": "01;00",
                "1197842700": "02;00",
                "1732036364": "00;03",
                "1471314444": "01;0.5",
                "1732798732": "02;0.5",
                "1729580556": "01;01",
                "1730039052": "1.5;01",
                "1730031884": "02;01",
                "1731958796": "2.5;01",
                "1463706636": "03;01",
                "1728351756": "01;1.5",
                "1730135820": "02;1.5",
                "1729890316": "01;02",
                "1470158860": "1.5;02",
                "1472705804": "02;02",
                "1466096396": "2.5;02",
                "1461592076": "03;02",
                "1462920972": "03;00",
                "1472246796": "01;03",
                "1733580300": "02;03",
                "1195669260": "03;03"
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

#mqttc.loop_forever()

##################################################################
################-- Path Planning--################################
def PathPlanning (Path,DestinationX, DestinationY):
	CurrentX = 0
	CurrentY = 0
	print("DestinationX: " + str(DestinationX))
	print("DestinationY: " + str(DestinationY))
	#go to x = 1, y = 0
	if (DestinationX % 1 == 0.5):
		print ("debug1")
		while (CurrentX  < DestinationX - 0.5):
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
		if (CurrentX < DestinationX):
			Path += "RS"
		print(Path)	
			
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
                print(Path)


destination = ""
## Button listeners 
def storeData0105(event):
    destination = "01;0.5"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))
    
def storeData0115(event):
    destination = "01;1.5"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))
    
def storeData1501(event):
    destination = "1.5;01"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))

def storeData1502(event):
    destination = "1.5;02"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))

def storeData0205(event):
    destination = "02;0.5"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))

def storeData0215(event):
    destination = "02;1.5"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))

def storeData2501(event):
    destination = "2.5;01"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))

def storeData2502(event):
    destination = "2.5;02"
    destinationX, destinationY = destination.split(';')
    PathPlanning (Path,float(destinationX), float(destinationY))
    returnPathPlanning(returnPath,float(destinationX), float(destinationY))

##Draw button map
def makeButtons():
    button0105 = plt.axes([0.35, 0.211, 0.06, 0.06 ])
    button0115 = plt.axes([0.35, 0.47, 0.06, 0.06 ])
    button1501 = plt.axes([0.48, 0.34, 0.06, 0.06])
    button1502 = plt.axes([0.48, 0.6, 0.06, 0.06])
    button0205 = plt.axes([0.615, 0.211, 0.06, 0.06])
    button0215 = plt.axes([0.615, 0.47, 0.06, 0.06])
    button2501 = plt.axes([0.74, 0.34, 0.06, 0.06])
    button2502 = plt.axes([0.74, 0.6, 0.06, 0.06])

    buttonDraw0105 = Button(button0105, "1,0.5")
    buttonDraw0115 = Button(button0115, "1,1.5")
    buttonDraw1501 = Button(button1501, "1.5,1")
    buttonDraw1502 = Button(button1502, "1.5,2")
    buttonDraw0205 = Button(button0205, "2,0.5")
    buttonDraw0215 = Button(button0215, "2,1.5")
    buttonDraw2501 = Button(button2501, "2.5,1")
    buttonDraw2502 = Button(button2502, "2.5,2")

##MARK: we print final destinations to console 
    buttonDraw0105.on_clicked(storeData0105)
    buttonDraw0115.on_clicked(storeData0115)
    buttonDraw1501.on_clicked(storeData1501)
    buttonDraw1502.on_clicked(storeData1502)
    buttonDraw0205.on_clicked(storeData0205)
    buttonDraw0215.on_clicked(storeData0215)
    buttonDraw2501.on_clicked(storeData2501)
    buttonDraw2502.on_clicked(storeData2502)


#########################Draw Path & Show Battery Level#######################
robotAX = ['01']
robotAY = ['00']
batteryA = []

def makeFig(): ##Create a function that makes plot
    plt.xlim(0, 3, 0.5)                        #Set x min and max values
    plt.ylim(0, 3, 0.5)                                 #Set y min and max values
    plt.title('Robot Path & Coordinates')         #Plot the title
    plt.grid(True) #Turn the grid on
    plt.ylabel('Y Coordinates')
    plt.xlabel('X Coordinates')  
    plt.plot(robotAX, robotAY, 'bs-', label='Robot Path')       #plot A coordinates 'bs-' marker = r'$\clubsuits$'
    plt.legend(loc= (0.75,1.02))     #plot the legend

    ##    Battery Level
    for label, x, y in zip(batteryA, robotAX, robotAY):
        plt.annotate(
        label, # some of these contain Emojis
        xy=(x, y), xytext=(-20, 20),
        textcoords='offset points', ha='right', va='bottom',
        bbox=dict(boxstyle='round,pad=0.2', fc='yellow', alpha=0.5),
        arrowprops=dict(arrowstyle = '->', connectionstyle='arc3,rad=0'),
        fontname='DejaVu Sans', # this is the param added
        fontsize=10)

def makeFigForButton():
    plt.xlim(0, 3, 0.5)                        #Set x min and max values
    plt.ylim(0, 3, 0.5)                                 #Set y min and max values
    plt.title('Final Destinations')         #Plot the title
    plt.grid(True) #Turn the grid on
    plt.ylabel('Y Coordinates')
    plt.xlabel('X Coordinates')

############################RFID Data Reading###############################
def convertCoordinates(coordinates):
    xa, ya = coordinates.rstrip().split(';')  #Split it into an array called dataArray
    robotAX.append(xa)
    print("XA:" + xa)
    robotAY.append(ya)
    print("YA:" + ya)
    if ((xa == '00') and (ya == '00')):                           
        print('This is the if')        
        for i in range(0, (len(robotAX) -1)):
            robotAX.pop(0)                       #This allows us to just see the last 50 data points
        for i in range(0, (len(robotAY) -1)):
            robotAY.pop(0)

#############################Switch Graphs###################################
def showButtons(*args, **kwargs):
    ## must draw again outside
    button0105 = plt.axes([0.35, 0.211, 0.06, 0.06 ])
    button0115 = plt.axes([0.35, 0.47, 0.06, 0.06 ])
    button1501 = plt.axes([0.48, 0.34, 0.06, 0.06])
    button1502 = plt.axes([0.48, 0.6, 0.06, 0.06])
    button0205 = plt.axes([0.615, 0.211, 0.06, 0.06])
    button0215 = plt.axes([0.615, 0.47, 0.06, 0.06])
    button2501 = plt.axes([0.74, 0.34, 0.06, 0.06])
    button2502 = plt.axes([0.74, 0.6, 0.06, 0.06])

    buttonDraw0105 = Button(button0105, "1,0.5")
    buttonDraw0115 = Button(button0115, "1,1.5")
    buttonDraw1501 = Button(button1501, "1.5,1")
    buttonDraw1502 = Button(button1502, "1.5,2")
    buttonDraw0205 = Button(button0205, "2,0.5")
    buttonDraw0215 = Button(button0215, "2,1.5")
    buttonDraw2501 = Button(button2501, "2.5,1")
    buttonDraw2502 = Button(button2502, "2.5,2")

	##MARK: we print final destinations to console 
    buttonDraw0105.on_clicked(storeData0105)
    buttonDraw0115.on_clicked(storeData0115)
    buttonDraw1501.on_clicked(storeData1501)
    buttonDraw1502.on_clicked(storeData1502)
    buttonDraw0205.on_clicked(storeData0205)
    buttonDraw0215.on_clicked(storeData0215)
    buttonDraw2501.on_clicked(storeData2501)
    buttonDraw2502.on_clicked(storeData2502)
    axnext = plt.axes([0.75, 0.9, 0.15, 0.075])
    bnext = Button(axnext, 'Show Path')
    bnext.on_clicked(path)

    b1 = ""

    plt.pause(0)

def path(*args, **kwargs):

    plt.ion() #Initiate map to plot live data

    ##MARK: read data via serial from Arduino - replace with MQTT
    global robotAX
    global robotAY
    global batteryA
    valid = True
    ##MARK: read data from arduino
    while valid: # while loop that loops forever
    ##            while (arduinoData.inWaiting() > 0): #Wait here until there is data
        UIDDataBase(UID);
        coordinates, batta = arduinoString.rstrip().split(';')
        batteryA.append(batta + 'V')
        
        drawnow(makeFig)    #Call drawnow to update our live graph
        plt.pause(.000001)  #Pause Briefly. Important to keep drawnow from crashing
        
        global bprev
        global b1
        print(robotAX)
        print(robotAY)
        if(robotAX[0] == '00' and robotAY[0] == '00'):
            robotAX.remove('00')
            robotAY.remove('00')
            robotAX = ['01']
            robotAY = ['00']
            valid = False
            plt.cla()
            makeFigForButton()
            axprev = plt.axes([0.12, 0.9, 0.2, 0.075])
            bprev = Button(axprev, 'Show Destination')
            bprev.on_clicked(showButtons)
            b1 = bprev
            break

        
while True:
    makeFigForButton()
    ## must draw again outside
    button0105 = plt.axes([0.35, 0.211, 0.06, 0.06 ])
    button0115 = plt.axes([0.35, 0.47, 0.06, 0.06 ])
    button1501 = plt.axes([0.48, 0.34, 0.06, 0.06])
    button1502 = plt.axes([0.48, 0.6, 0.06, 0.06])
    button0205 = plt.axes([0.615, 0.211, 0.06, 0.06])
    button0215 = plt.axes([0.615, 0.47, 0.06, 0.06])
    button2501 = plt.axes([0.74, 0.34, 0.06, 0.06])
    button2502 = plt.axes([0.74, 0.6, 0.06, 0.06])

    buttonDraw0105 = Button(button0105, "1,0.5")
    buttonDraw0115 = Button(button0115, "1,1.5")
    buttonDraw1501 = Button(button1501, "1.5,1")
    buttonDraw1502 = Button(button1502, "1.5,2")
    buttonDraw0205 = Button(button0205, "2,0.5")
    buttonDraw0215 = Button(button0215, "2,1.5")
    buttonDraw2501 = Button(button2501, "2.5,1")
    buttonDraw2502 = Button(button2502, "2.5,2")

	##MARK: we print final destinations to console 
    buttonDraw0105.on_clicked(storeData0105)
    buttonDraw0115.on_clicked(storeData0115)
    buttonDraw1501.on_clicked(storeData1501)
    buttonDraw1502.on_clicked(storeData1502)
    buttonDraw0205.on_clicked(storeData0205)
    buttonDraw0215.on_clicked(storeData0215)
    buttonDraw2501.on_clicked(storeData2501)
    buttonDraw2502.on_clicked(storeData2502)
	mqttc.publish("Path", Path+returnPath, mid)
		
    
    axnext = plt.axes([0.75, 0.9, 0.15, 0.075])
    bnext = Button(axnext, 'Show Path')
    bnext.on_clicked(path)

    b1 = ""
    plt.show()

plt.show()









    

