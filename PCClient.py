#Quadcopter UAV Project
#Ground control station - Spring 2020
#Andrew Cisneros

from tkinter import *
import socket, threading
import time, pickle, cv2
from PIL import Image
from PIL import ImageTk
import struct
import numpy as np

#NETWORK details
# 2 sockets are created - both TCP

serverIP = '192.168.4.1'  #static ip of drone
videoPort = 1205
commPort = 1200
videoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
commSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    #specify ipv4 and TCP
videoSocket.connect((serverIP, videoPort))
commSocket.connect((serverIP, commPort)) 

print('Connected to drone on ' + serverIP)

#Retieve facial image classifiers
face_cascade = cv2.CascadeClassifier("C:\Program Files\Python37\Lib\site-packages\cv2\data\haarcascade_frontalface_default.xml")
eye_cascade = cv2.CascadeClassifier("C:\Program Files\Python37\Lib\site-packages\cv2\data\haarcascade_eye.xml")
smile_cascade = cv2.CascadeClassifier("C:\Program Files\Python37\Lib\site-packages\cv2\data\haarcascade_smile.xml")
palm_cascade = cv2.CascadeClassifier("C:\Program Files\Python37\Lib\site-packages\cv2\data\haarcascade_palm.xml")
fist_cascade = cv2.CascadeClassifier("C:\Program Files\Python37\Lib\site-packages\cv2\data\haarcascade_fist.xml")
body_cascade = cv2.CascadeClassifier("C:\Program Files\Python37\Lib\site-packages\cv2\data\haarcascade_fullbody.xml")

#Define Class for app
class DroneGroundControl():
    def __init__(self, window):
        #String variables for app
        self.altStr = StringVar()
        self.entryAlt = StringVar()
        self.etcStr = StringVar()
        self.objectStr = StringVar()
        
        #create widgets/frames
        self.videoCanvas = Label(window, relief='raised')
        self.takeOff_frame = Frame(window)
        self.withInView_frame = Frame(window)
        self.etc_frame = Frame(window)
        self.navigation_frame = Frame(window, relief='raised')

        #Takeoff/altitude control --  create button/labels/ entrys for altitude control/ takeoff widget
        self.currentAlt_label = Label(self.takeOff_frame, text = "Current Altitude: ")
        self.displayAlt_label = Label(self.takeOff_frame, textvariable= self.altStr)

        self.meters_label = Label(self.takeOff_frame, text = "Meters")
        self.meters_label2 = Label(self.takeOff_frame, text = "Meters")

        self.title_label = Label(self.takeOff_frame, text = "Altitude Control Widget", anchor= CENTER, font= "12")
        #THROTTLE buttons --ascend / descend
        self.ascend_button = Button(self.takeOff_frame, text= "^ Ascend", command = self.sendAscend)
        self.descend_button = Button(self.takeOff_frame, text= "v Descend", command = self.sendDescend)

        self.arm_button = Button(self.takeOff_frame, text = "Arm!", width = 8, height = 4, command= self.sendArm)
        self.land_button = Button(self.takeOff_frame, text = "Land!", width =8, height = 4, command = self.sendLand)

        self.targetAlt_button = Button(self.takeOff_frame, text="Take Off:", command = self.sendTakeOff)
        self.targetAlt_entry = Entry(self.takeOff_frame,textvariable= self.entryAlt)

        #Objects in view  -- create labels for objects with in view widget
        self.recObjects_label = Label(self.withInView_frame, text = "Objects Recognized")
        self.objects_label = Label(self.withInView_frame, width = 20, height =16, relief='raised', textvariable = self.objectStr)
        #Create Application Logo
        self.logo_label = Label(window, text = "Drone Control", bg="light blue", fg="black", font="Verdana 32 bold italic")

        #Extaneous messages -- create label for etc. messages recieved from drone
        self.etc_title_label = Label(self.etc_frame, text = "Extraneous messages from vehicle:")
        self.etc_label = Label(self.etc_frame, textvariable = self.etcStr, relief= 'raised', height= 8, width= 27)

        #CREATE buttons for navigation frame/widget
        #widget label/title
        self.navTitle_label = Label(self.navigation_frame, text = "Navigation Widget", anchor= CENTER, font= "12")
        #YAW
        self.rotateC_button = Button(self.navigation_frame, text = "Rotate Clockwise", command = self.sendClockwise)
        self.rotateCC_button = Button(self.navigation_frame, text = "Rotate Counter-Clockwise", command = self.sendCounterClockwise)
        #PITCH
        self.forward_button = Button(self.navigation_frame, text = "^", width = 5, height = 3, command = self.sendForward)
        self.backward_button = Button(self.navigation_frame, text = "v", width = 5, height= 3, command= self.sendBackward)
        #ROLL
        self.left_button = Button(self.navigation_frame, text = "<", width=5, height=3, command = self.sendLeft)
        self.right_button = Button(self.navigation_frame, text = ">", width=5, height=3, command= self.sendRight)

        
        #PLACE objects in window
        #video canvas
        self.logo_label.grid(row=0, column=0)
        self.videoCanvas.grid(row=1, column=0)
        #take 0ff widget
        self.takeOff_frame.grid(row=4, column=0, padx=12)
        self.title_label.grid(row=0, columnspan=5)
        self.currentAlt_label.grid(row=1, column=0)
        self.displayAlt_label.grid(row=1, column=1)
        self.meters_label2.grid(row=1, column=2)

        self.ascend_button.grid(row=2,column=0)
        self.descend_button.grid(row=2, column=1)
        self.land_button.grid(row=3, column = 1)
        self.arm_button.grid(row=3, column=0)
        self.targetAlt_button.grid(row=3, column=2)
        self.targetAlt_entry.grid(row=3, column=3)
        self.meters_label.grid(row=3, column=4)
        
        #Place objects within view frame
        #Objects in view frame
        self.withInView_frame.grid(row=1, column=1, pady =2)
        self.recObjects_label.grid(row=0, columnspan=2)
        self.objects_label.grid(row=1, columnspan=2)

        #place ETC frame for extraneous messages from drone
        self.etc_frame.grid(row=1,column=2,pady=2 )
        self.etc_title_label.grid(row=0, columnspan =2)
        self.etc_label.grid(row=1, columnspan=2)

        #place navigation frame
        self.navigation_frame.grid(row=4, column=1, padx=15)
        self.navTitle_label.grid(row=0, columnspan =3)
        
        self.rotateC_button.grid(row=1, column=0) 
        self.rotateCC_button.grid(row=1, column=2)
        
        self.forward_button.grid(row=2, column=1)
        self.backward_button.grid(row=4, column=1)
        
        self.left_button.grid(row=3, column=0)
        self.right_button.grid(row=3, column=2)
        
        
        #start streaming video in seperate thread
        videoThread = threading.Thread(target = self.updateVideo)
        videoThread.start()
        #listen/recv messages in sperate thread
        recvMsgThread = threading.Thread(target = self.recvMessages)
        recvMsgThread.start()

    #Purpose: receives video stream and displays in tkinter GUI
    #Also runs objects detection code
    def updateVideo(self):
            data = b''  #byte literal
            payload_size = struct.calcsize("L")
            print("Video stream started")

            numSmiles = 0
            numEyes = 0
            numFaces = 0
            numPalms = 0
            numFists = 0
            
            while True:
                #retrieve message size
                while len(data)< payload_size:
                    data += videoSocket.recv(4096)

                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("L", packed_msg_size)[0]

                while len(data) < msg_size:
                    data += videoSocket.recv(4096)

                frame_data = data[:msg_size]
                data = data[msg_size:]

                #extract frame and format
                frame = pickle.loads(frame_data, fix_imports=True, encoding = 'bytes')
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

                ##HERE we have the frame
                #convert frame to grayscale for object detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


                ##################
                #BeginObject Detection:
                
                #Square color chart
                #Faces: blue
                #Eyes: green
                #Smile: purple
                #Open hand: red
                #Closed Hand: cyan
               

                #create detector for faces
                faces = face_cascade.detectMultiScale(gray, 1.3, 5)
                #get Number of faces
                numFaces = len(faces)
                
                #Loop to draw blue rectangle
                for (x,y,w,h) in faces:
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2) ##255 bit color. BGR format
                    roi_gray = gray[y:y+h, x:x+w]
                    roi_color = frame[y:y+h, x:x+w]

                    #Now within the same area faces are found, search for smiles and eyes
                    #Searching only the face for facial feature reduces false positives and reduces resources
                    
                    #create detector for smiles    
                    smiles = smile_cascade.detectMultiScale(roi_gray)
                    #get number smiles
                    numSmiles = len(smiles)
                    for (sx,sy,sw,sh) in smiles:
                        cv2.rectangle(roi_color,(sx,sy),(sx+sw,sy+sh),(128,0,128),2)

                    #create detector for eyes    
                    eyes = eye_cascade.detectMultiScale(roi_gray)
                    #get number of eyes found
                    numEyes = len(eyes)
                    for (ex,ey,ew,eh) in eyes:
                        cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2) #green rectangles

                #create detector for palms
                palms = palm_cascade.detectMultiScale(gray, 1.3, 5)
                numPalms = len(palms)
                for (x,y,w,h) in palms:
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
                    
                #create detector for fists
                fists = fist_cascade.detectMultiScale(gray, 1.3, 5)
                numFists = len(fists)
                for (x,y,w,h) in fists:
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),2)
               
                #updates the number of objects seen on screen
                self.updateObjects(numFaces, numSmiles, numEyes, numPalms, numFists)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                #display frame
                photo = ImageTk.PhotoImage(image = Image.fromarray(frame))
                self.videoCanvas.configure(image = photo)
                self.videoCanvas.image = photo
                
##The following member functions send movement commands to the drone
                
    def sendAscend(self):
        print("Sent Ascend Message")
        commSocket.send(("Up").encode())

    def sendDescend(self):
        print("Sent Descend Message")
        commSocket.send(("Down").encode())

    def sendForward(self):
        print("Sent Forward Message")
        commSocket.send(("^").encode())
    
    def sendBackward(self):
        print("Sent Backward Message")
        commSocket.send(("v").encode())

    def sendLeft(self):
        print("Sent Left Message")
        commSocket.send(("<").encode())

    def sendRight(self):
        print("Sent Right Message")
        commSocket.send((">").encode())

    def sendClockwise(self):
        print("Sent Clockwise Message")
        commSocket.send(("c").encode())

    def sendCounterClockwise(self):
        print("Sent Counter-Clockwise Message")
        commSocket.send(("cc").encode())

    #Purpose: sends takeoff command to drone with alititude to ascend to (in meters)
    def sendTakeOff(self):
        alt = self.entryAlt.get()
        msg = "T" + alt
        print(msg)
        commSocket.send(msg.encode())
        
    #Purpose: send arm command to quad
    def sendArm(self):
        arm = "Arm"
        commSocket.send(arm.encode())
        print("Arm msg sent")
    #Purpose: sends land command
    def sendLand(self):
        land = "Land"
        commSocket.send(land.encode())

    #Purpose: Updates the list of visable objects 
    def updateObjects(self, numFaces, numSmiles, numEyes, numPalms, numFists):

        updateStr = ""
        
        if (numFaces > 0):
            updateStr = "Faces Detected: " + str(numFaces)
        if(numSmiles > 0):
            updateStr= updateStr + "\n" + "Smiles Detected: " + str(numSmiles)
        if(numEyes > 0):
            updateStr= updateStr + "\n" + "Eyes Detected: " + str(numEyes)
        if(numPalms > 0):
            updateStr= updateStr + "\n" + "Open Hands Detected: " + str(numPalms)
        if(numFists > 0):
            updateStr= updateStr + "\n" + "Closed Hands Detected: " + str(numFists)
        if(numFaces == 0 and numSmiles == 0 and numEyes ==0 and numPalms ==0 and numFists ==0):
            updateStr = "No object recognized"

        self.objectStr.set(updateStr)
        
    #Purpose: function runs in seperate thread constantly recving and processing messages from the drone
    def recvMessages(self):
        while 1:
            msg = commSocket.recv(1024).decode()
           # message header for altitude message; display in current altitude label
            if(msg[0] == '@'):
                msg = msg.replace('@', '')
                self.altStr.set(msg)

            #The message is extraneous; display it on the etc. string 
            else:
                self.etcStr.set(msg)
            

#create window instance and configure
window = Tk()
window.title("QuadCopter Ground Control")
window.configure(background='light blue')
window.geometry('800x600')           

#create instance of app
app = DroneGroundControl(window)

window.mainloop()

