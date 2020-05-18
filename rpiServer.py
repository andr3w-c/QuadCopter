
#Andrew Cisneros
#Program runs onboard a raspberry pi zero w. 
#Program captures video on a RPI camera module and streams over an ad-hoc wifi network to a connected PC
#Program also recieves flight commands from client PC


# import necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video.pivideostream import PiVideoStream
import imutils
import time, math
import socket
import struct
import pickle
import cv2

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

import threading

#port number our application will run on
videoPort =  1205
comPort = 1200

#Create 2 TCP sockets: 1 for video stream. 1 for all other communication
#sockets are called videoSocket and commSocket respectivley
#video feed runs on it own TCP socket, reducing network traffic
try:
	#socket to listen for  video soc connection
        SocketA = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	#socket to listen for defualt communication
	SocketB = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except:
        print('Failed to create socket, exiting...')
        exit()
else:
        print('Sockets created')
        SocketA.bind(('', videoPort))
	SocketB.bind(('', comPort))
        SocketA.listen(1)   #allow one connection
        SocketB.listen(1)

        videoSocket, addr = SocketA.accept()
	commSocket, addr = SocketB.accept()
        print('connected to ' + addr[0])
#wi-fi connection  Established####

#Connect to the Vehicle / pixhawk 4 mini with a baud rate of 57,600 bit/s
print('Connecting to vehicle on: serial0')
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)
print("connected to Pixhawk!")


#Purpose: This function is responsible for capturing video from the rip cam module and transmiting frame data over wifi
def videoStream():
	##PiVideoStream starts camera capture continous in a seperate thread
	#Keeping call to I/O in seperate thread keeps our main loop uniterupted and acheives higher FPS
	encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
	vs = PiVideoStream().start()
	time.sleep(1.0)
	print("Capturing video...")
	#keep reading frames
	try:
		while True:
			frame = vs.read()
			#compress image - allows for faster live stream
			ret, frame = cv2.imencode('.jpg',frame, encode_param)

			#View stream on RPI HDMI
			#cv2.imshow("frame", frame)
			#cv2.waitKey(1) & 0xFF

			#Now serialize the data with pickle
			data = pickle.dumps(frame)
			#add msgLength to a struct- send message length first, then data
			message_size = struct.pack("L", len(data))
			#SEND msg length + DATA
			videoSocket.sendall(message_size + data)
	except:
		vs.stop()
		videoSocket.close()
		commSocket.close()
		exit()

#Purpose:
def takeoff_nogps(aTargetAltitude):
    """
    Launches vehicle to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    #if not armed,  stop
    if(vehicle.armed == False):
	print("Vehicle not armed")
	commSocket.send(("not armed").encode())
        return None

    #set thrust value until vehicle has reached target alltitude
    print("Taking off!")
    commSocket.send(("Launching!").encode())
    starting_altitude = vehicle.location.global_relative_frame.alt
    TargetAltitude = aTargetAltitude + starting_altitude
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude-starting_altitude, aTargetAltitudee))
        if current_altitude >= TargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude < aTargetAltitude:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)



def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
#    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
 #                 When one is used, the other is ignored by Ardupilot.
  #  thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

#Purpose: Allows the vehicles navigation control. set roll, pitch, yaw and thrust
def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):

    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)
    
#Purpose: Convert degrees to quaternions
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    #Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

#Purpose: Function runs in a seperate thread to constantly listen to messages from PC client and process them
def recvMessages():
    print("Recieve msg Thread created:: ")
    while 1:
           try:
               message =  commSocket.recv(1024).decode()
           except:
                 commSocket.close()
		 videoSocket.close()
                 print("connectionlost")
		 exit()
           else:

		#Process commands from Groud Station PC
		#Command is to takeoff in the format T<int>, int is the hight to ascend to
                if(message[0] == 'T'):
                  print("Take off command recieved!")
                  #Take off in GUIDED_NOGPS mode
		  #extract meters to ascend to
                  num = float(message[1])
                  takeoff_nogps(num) ##pass altitude
                  print("Hold height")
		  #Hold altitude 
                  time.sleep(1)

		#Command is to Arm the copter. Arm the vehicle. (prepare for takeoff)
		elif(message == "Arm"):
		   print("Attempting to arm motors")
                   #Copter should arm in GUIDED_NOGPS mode
                   vehicle.mode = VehicleMode("GUIDED_NOGPS")

                   vehicle.armed = True
                   time.sleep(1)
		   #Inform the client PC the status of the vehicle
		   if(vehicle.armed == True):
                      print("Armed")
  	              commSocket.send(("Armed").encode())
		   else:
		       print("could not arm")
                       commSocket.send(("Not Armed").encode())

		#Message was Land --  Land the vehicle
		elif(message == "Land"):
                	vehicle.mode = VehicleMode("LAND")
			commSocket.send(("Landing").encode())
			print("Landing")

		#Message was UP -- Ascend for 1 second at a thrust level of 0.6
		elif(message == "Up"):
			print("Ascend")
			set_attitude(thrust = 0.6, duration = 1)
		#Message was DOWN -- Descend for 1 second at thrust level of 0.4
		elif(message == "Down"):
			print("Descend")
			set_attitude(thrust = 0.4, duration = 1)
		#Message was ^ -- Pitch/move forward for 1 second
		elif(message == "^"):
			print("Forward")
			set_attitude(pitch_angle = -5, thrust = 0.5, duration = 1)
		#Message was v -- Pitch/move backward for 1 second
		elif(message == "v"):
			print("Backward")
			set_attitude(pitch_angle = 5, thrust = 0.5, duration =1)
		#Message was < -- Roll/move left for 1 second
		elif(message == "<"):
			print("Left")
			set_attitude(roll_angle = -5, thrust = 0.5, duration =1)
		#Message was > -- Roll/move right for 1 second
		elif(message == ">"):
			print("Right")
			set_attitude(roll_angle = 5, thurst = 0.5, duration =1)
		#Message was c -- Yaw clockwise for 1 second
		elif(message == "c"):
			print("Yaw Clockwise")
			set_attitude(yaw_rate = 20, thrust = 0.5, duration=1)
		#Message was cc -- Yaw counter-clockwise for 1 second
		elif(message == "cc"):
			print("Yaw Counter-Clockwise")
			set_attitude(yaw_rate = -20, thrust = 0.5, duration = 1)

                else:
                    print("Unknown message recieved")

#Purpose: Retrieves altitude reading from onboard barometer on pixhawk
# and sends it to the PC ground station every 1s
def update_altitude():

    while 1:
    	alt = vehicle.location.global_relative_frame.alt
    	alt = str(alt)
	alt = "@" + alt
    	commSocket.send(alt.encode())
    	time.sleep(1)






videoThread = threading.Thread(target= videoStream)
videoThread.start()

recvMsgThread = threading.Thread(target = recvMessages)
recvMsgThread.start()

altThread = threading.Thread(target = update_altitude)
altThread.start()
