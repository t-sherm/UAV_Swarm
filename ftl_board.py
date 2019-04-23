# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 18:49:20 2019

@author: UAV Lab Computer
Tristan Sherman

Cool stuff UAV Swarm Cal Poly Pomona 2019
"""
from __future__ import print_function
import ftl_definitions
from ftl_definitions import *
import time
from struct import *
from RF24 import *
from RF24Network import *
import sys
import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import threading

#Semaphore Declaration
semaphore = threading.Semaphore()

def start_network():
	#Variable Initialization
	
	#Recieve Variables
	global r_lat
	global r_lon
	global r_alt
	global r_hdg
	
	#Send Variables
	global s_lat
	global s_lon
	global r_alt
	global r_hdg
	
	#Radio Configuration
	#Configure Radio SPI Connections
	radio = RF24(25,0)
	network = RF24Network(radio)
	
	#Octal
	octlit = lambda n:int(n,8)
	
	#Read sid.txt for network address information
	lines = [line.rstrip('\n') for line in open('sid.txt')]
	l_sid = lines[0]
	o_sid = lines[1]
	this_node = octlit(l_sid)
	other_node = octlit(o_sid)
	print('This Node: ', this_node, ' Other Node: ', other_node)
	
	#Start Radio
	radio.begin()
	time.sleep(0.1)
	
	#Open network on channel 90
	network.begin(90, this_node)
	time.sleep(0.1)
	radio.printDetails()
	
	#Initialize packet info
	packets_sent = 0
	packets_recieved = 0	

	#Send and Recieve Loop
	while 1:
		#Update Network
		network.update()
		print('Packets Sent: ', packets_sent, ' Packets Recieved: ', packets_recieved)

		#Send Information
	
		semaphore.acquire()
		#..CRITICAL REGION
		payload = pack('<ffff', s_lat, s_lon, s_alt, s_hdg)
		#..END CRITICAL REGION
        	semaphore.release()
		
		ok = network.write(RF24NetworkHeader(other_node), payload)
        	if ok:
        		#print('Successfully Sent Data')
            		packets_sent += 1
        	else:
            		print('FAILED To Send Data')
		
		#Retrieve Incoming Packets
		while network.available():
            		header, o_payload = network.read(16)
            		lat, lon, alt, hdg = unpack('<ffff', bytes(o_payload))
         		print('Recieved Payload', ' Lat: ', lat, ' Lon: ', lon, ' Alt:', alt, ' Hdg: ', hdg, ' | From ', oct(header.from_node))
    		
			semaphore.acquire()
			#..CRITICAL REGION        
			r_lat = lat
            		r_lon = lon
            		r_alt = alt
            		r_hdg = hdg
			#..END CRITICAL REGION
			semaphore.release()
			
            		packets_recieved += 1
		time.sleep(0.5)


# Startup
# Connect to vehicle
vehicle = connect('/dev/serial0', wait_ready=True, baud = 57600)
# Send ack of successful connect
# ---------- TODO -----------------
# Tristan Cady: make a function that sends an ack

r_lat = 1
r_lon = 1
r_alt = 1
r_hdg = 1
s_lat = 1
s_lon = 1
s_alt = 1
s_hdg = 1

# Desired parameters
flightAltitude = 10 #meters

#Start Networking Thread
nt = threading.Thread(target=start_network, args=()) 
nt.start()

safetySw = 1600

# Wait for receipt of messages
# ------------ TODO -------------
# Tristan Cady: Somehow have the code wait until we make sure the connection to the other computer is secured

boid = [1]*2
boid[0] = Boid('ourBoid',[ 1, 0, 0], [0, 0, 0])
boid[1] = Boid('otherBoid', [ 0, 1, 0], [0, 0, 0])


print("TEST")
# While loop ------------------------
while True:
    
    if safetySw > 1500:

        # Calculatie new position
        # Get velocity and position of the controlled boid
        boid[0].velocity = vehicle.velocity
        boid[0].position = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt] 
        print(boid[0].position[0])
        print(boid[0].position[1])
        print("Leader")
        print(boid[1].position[0])
        print(boid[1].position[1])
        print(boid[1].hdg)
        # Obtain the heading value of the controlled boid
        boid[0].hdg = vehicle.heading

    
        # ---------------------- TODO ------------------------
        # EVENTUALLY: Determine which boid is the local leader
        
        
        #----------------------- TODO --------------------------
        # TRISTAN CADY: Obtain position, velocity, and heading of other drone
        # Obtain position, velocity and heading of the other boid
	
	semaphore.acquire()
	#SET SEND VARIABLES
	s_lat = boid[0].position[0]
	s_lon = boid[0].position[1]
	s_alt = boid[0].position[2]
	s_hdg = boid[0].hdg
	#SET RECIEVED VARIABLES
	boid[1].position[0] = r_lat
	boid[1].position[1] = r_lon
	boid[1].position[2] = r_alt
	boid[1].hdg = r_hdg
	semaphore.release()

	print('Lat:',boid[1].position[0],' Lon:',boid[1].position[1])	
        #Calculate steering vector for the boid
        cohesionVector = ftl_definitions.calculateCohesionVector(boid,0)
        separationVector = ftl_definitions.calculateSeparationVector(boid,0)
        velocity = ftl_definitions.updateVelocity(boid,1,cohesionVector, separationVector)

        print('velocity[0]: lat: ', velocity[0])
        print('velocity[1]: lon: ', velocity[1])
        print('velocity[3]: heading: ', velocity[3])
        
        # Command a velocity to the boid
        ftl_definitions.send_ned_velocity(vehicle, velocity[0], velocity[1], velocity[2], 1)
       
        # Command heading (rotation/yaw)
        command_yaw(vehicle, velocity[3], relative=False)   
        
        
        
    else:
        print("Code stalled. Waiting for safety switch to reactivate")
        vehicle.mode = VehicleMode("STABILIZE")
    
    time.sleep(1)
