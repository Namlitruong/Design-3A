import serial # import Serial Library
import matplotlib.pyplot as plt #import matplotlib library

import warnings
import matplotlib.cbook

from matplotlib.widgets import Button
from matplotlib.font_manager import FontProperties
from matplotlib.offsetbox import AnnotationBbox, OffsetImage


warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)

uid = ""

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

    
plt.xlim(0, 3, 0.5)                        #Set x min and max values
plt.ylim(0, 3, 0.5)                                 #Set y min and max values
plt.title('Robot Path & Coordinates')         #Plot the title
plt.grid(True) #Turn the grid on
plt.ylabel('Y Coordinates')
plt.xlabel('X Coordinates')  

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

buttonDraw0105.on_clicked(storeData0105)
buttonDraw0115.on_clicked(storeData0115)
buttonDraw1501.on_clicked(storeData1501)
buttonDraw1502.on_clicked(storeData1502)
buttonDraw0205.on_clicked(storeData0205)
buttonDraw0215.on_clicked(storeData0215)
buttonDraw2501.on_clicked(storeData2501)
buttonDraw2502.on_clicked(storeData2502)

plt.show()
