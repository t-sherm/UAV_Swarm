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

# Startup
# Connect to vehicle
vehicle = connect('/dev/serial0', wait_ready=True, baud = 57600)
# Send ack of successful connect
# ---------- TODO -----------------
# Tristan Cady: make a function that sends an ack

# Radio Configuration
# Configure Radio SPI Connections
radio = RF24(25,0)
network = RF24Network(radio)

# Octal
octlit = lambda n:int(n, 8)

# Pull information about network address
lines = [line.rstrip('\n') for line in open('sid.txt')]

l_sid = lines[0]
o_sid = lines[1]

this_node = octlit(l_sid)
other_node = octlit(o_sid)
print('This Node: ', this_node, ' Other Node: ', other_node)

radio.begin()
time.sleep(0.1)

# Open network on channel 90
network.begin(90, this_node)
radio.printDetails()

# Initialize packet info
packets_sent = 0
packets_recieved = 0

# Wait for toggle from aircraft
# Takeoff to 10m
safetySw = 1600
while safetySw < 1500:
    
    safetySw = vehicle.channels['5']
    print (" Ch5: %s" % safetySw)
    

# Wait for receipt of messages
# ------------ TODO -------------
# Tristan Cady: Somehow have the code wait until we make sure the connection to the other computer is secured

boid = [1]*2
boid[0] = Boid('ourBoid',[ 1, 0, 0], [0, 0, 0])
boid[1] = Boid('otherBoid', [ 0, 1, 0], [0, 0, 0])

# While loop ------------------------
while True:
    
    if safetySw > 1500:

        # Update network
        network.update()
        print('Packets Sent: ', packets_sent, ' Packets Recieved: ', packets_recieved)

        # Calculatie new position
        # Get velocity and position of the controlled boid
        boid[0].velocity = vehicle.velocity
        boid[0].position = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt] 
        print(boid[0].position[0])
        print(boid[0].position[1])
        # Obtain the heading value of the controlled boid
        boid[0].hdg = vehicle.heading

        payload = pack('<ffff', boid[0].position[0], boid[0].position[1], boid[0].position[2], boid[0].hdg)
        ok = network.write(RF24NetworkHeader(other_node), payload)
        if ok:
            print('Successfully Sent Data')
            packets_sent += 1
        else:
            print('FAILED To Send Data')
    
        # ---------------------- TODO ------------------------
        # EVENTUALLY: Determine which boid is the local leader
        
        
        #----------------------- TODO --------------------------
        # TRISTAN CADY: Obtain position, velocity, and heading of other drone
        # Obtain position, velocity and heading of the other boid
        while network.available():
            header, o_payload = network.read(16)
            lat, lon, alt, hdg = unpack('<ffff', bytes(o_payload))
            print('Recieved Payload', ' Lat: ', lat, ' Lon: ', lon, ' Alt:', alt, ' Hdg: ', hdg, ' | From ', oct(header.from_node))
            boid[1].position[0] = lat
            boid[1].position[1] = lon
            boid[1].position[2] = alt
            boid[1].hdg = hdg
            packets_recieved += 1

        #Calculate steering vector for the boid
        cohesionVector = ftl_definitions.calculateCohesionVector(boid,0)
        separationVector = ftl_definitions.calculateSeparationVector(boid,0)
        velocity = ftl_definitions.updateVelocity(boid,1,cohesionVector, separationVector)
        
        # Command a velocity to the boid
        ftl_definitions.send_ned_velocity(vehicle, velocity[0], velocity[1], velocity[2], 1)
       
        # Command heading (rotation/yaw)
        command_yaw(vehicle, velocity[3], relative=False)   
        
        
        
    else:
        print("Code stalled. Waiting for safety switch to reactivate")
        
    
    time.sleep(1)
