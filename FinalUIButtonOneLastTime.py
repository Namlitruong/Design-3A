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

warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)

###################Draw Buttons to Send Final Destination####################
## global uid
uid = ""

## Button listeners 
def storeData0105(event):
    uid = "01;0.5"
    print(uid)
    
def storeData0115(event):
    uid = "01;1.5"
    print(uid)
    
def storeData1501(event):
    uid = "1.5;01"
    print(uid)

def storeData1502(event):
    uid = "1.5;02"
    print(uid)

def storeData0205(event):
    uid = "02;0.5"
    print(uid)

def storeData0215(event):
    uid = "02:1.5"
    print(uid)

def storeData2501(event):
    uid = "2.5;1.0"
    print(uid)

def storeData2502(event):
    uid = "2.5;02"
    print(uid)

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

##Turn uid in String for path recognition on UI 
def ReadRFIDInCoor(arduinoString): 
    uid = arduinoString
    int00 = '1205479948'
    int10 = '1473313548'
    int20 = '1197842700'
    int03 = '1732036364' 
    int105 = '1471314444'
    int205 = '1732798732'
    int11 = '1729580556'
    int151 = '1730039052'
    int21 = '1730031884'
    int251 = '1731958796'
    int31 = '1463706636'
    int115 = '1728351756' 
    int215 = '1730135820'
    int12 = '1729890316'
    int152 = '1470158860'
    int22 = '1472705804' 
    int252 = '1466096396'
    int32 = '1461592076' 
    int30 = '1462920972'
    int13 = '1472246796' 
    int23 = '1733580300'
    int33 = '1195669260'
    
    if (uid == int00):
        convertCoordinates("00;00")
        print("00;00")
    elif (uid == int10):
        convertCoordinates("01;00")
        print("01;00")
    elif (uid == int20):
        convertCoordinates("02;00")
        print("02;00")
    elif (uid == int03):
        convertCoordinates("00;03")
        print("00;03")
    elif (uid == int105):
        convertCoordinates("01;0.5")
        print("01;0.5")
    elif (uid == int205):
        convertCoordinates("02;0.5")
        print("02;0.5")
    elif (uid == int11):
        convertCoordinates("01;01")
        print("01;01")
    elif (uid == int151):
        convertCoordinates("1.5;01")
        print("1.5;01")
    elif (uid == int21):
        convertCoordinates("02;01")
        print("02;01")
    elif (uid == int251):
        convertCoordinates("2.5;01")
        print("2.5;01")
    elif (uid == int31):
        convertCoordinates("03;01")
        print("03;01")
    elif (uid == int115):
        convertCoordinates("01;1.5")
        print("01;1.5")
    elif (uid == int215):
        convertCoordinates("02;1.5")
        print("02;1.5")
    elif (uid == int12):
        convertCoordinates("01;02")
        print("01;02")
    elif (uid == int152):
        convertCoordinates("1.5;02")
        print("1.5;02")
    elif (uid == int22):
        convertCoordinates("02;02")
        print("02;02")
    elif (uid == int252):
        
        convertCoordinates("2.5;02")
        print("2.5;02")
    elif (uid == int32):
        convertCoordinates("03;02")
        print("03;02")
    elif (uid == int30):
        convertCoordinates("03;00")
        print("03;00")
    elif (uid == int13):
        convertCoordinates("01;03")
        print("01;03")
    elif (uid == int23):
        convertCoordinates("02;03")
        print("02;03")
    elif (uid == int33):
        convertCoordinates("03;03")
        print("03;03")
    else:
        print("new coordinates")


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
    arduinoData = serial.Serial('/dev/ttyACM0', 9600)
    global robotAX
    global robotAY
    global batteryA
    valid = True
    ##MARK: read data from arduino
    while valid: # while loop that loops forever
    ##            while (arduinoData.inWaiting() > 0): #Wait here until there is data
        arduinoString = arduinoData.readline() #read the line of text from the serial port
        arduinoString = arduinoString.rstrip('\n')
        arduinoString = arduinoString[:-1]
        coordinates, batta = arduinoString.rstrip().split(';')
        batteryA.append(batta + 'V')
        ReadRFIDInCoor(coordinates)

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

    axnext = plt.axes([0.75, 0.9, 0.15, 0.075])
    bnext = Button(axnext, 'Show Path')
    bnext.on_clicked(path)

    b1 = ""
    plt.show()

plt.show()





    
