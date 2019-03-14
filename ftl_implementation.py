# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 18:49:20 2019

@author: UAV Lab Computer
Tristan Sherman

Cool stuff UAV Swarm Cal Poly Pomona 2019
"""

import ftl_definitions
from ftl_definitions import *

import sys
import dronekit
from dronekit import connect

# Startup
# Connect to vehicle
vehicle = connect('/dev/serial0', wait_ready=True, baud = 57600)
# Send ack of successful connect
# ---------- TODO -----------------
# Tristan Cady: make a function that sends an ack

# Wait for toggle from aircraft
# Takeoff to 10m
safetySw = 0
while safetySw < 1500:
    
    safetySw = vehicle.channels['5']
    print (" Ch5: %s" % safetySw)
    

# Wait for receipt of messages
# ------------ TODO -------------
# Tristan Cady: Somehow have the code wait until we make sure the connection to the other computer is secured

boid = [1]*2
boid[0] = Boid('ourBoid',[ 0, 0, 0], [0, 0, 0])
boid[1] = Boid('otherBoid', [ 0, 0, 0], [0, 0, 0])

# While loop ------------------------
while True:
    
    if safetySw > 1500:
        
        #Calculatie new position      
        # Get velocity and position of the controlled boid
        boid[0].velocity = vehicle.velocity
        boid[0].position = vehicle.location.global_relative_frame
            
        # Obtain the heading value of the controlled boid
        boid[0].hdg = vehicle.heading
        
    
        # ---------------------- TODO ------------------------
        # EVENTUALLY: Determine which boid is the local leader
        
        
        #----------------------- TODO --------------------------
        # TRISTAN CADY: Obtain position, velocity, and heading of other drone
        # Obtain position, velocity and heading of the other boid
        
        
        #Calculate steering vector for the boid
        cohesionVector = calculateCohesionVector(boid[0],0)
        separationVector = calculateSeparationVector(boid[0],0)
        velocity = updateVelocity(boid[0],1,cohesionVector, separationVector)
        
        # Command a velocity to the boid
        send_ned_velocity(vehicle, velocity[0], velocity[1], [2], 1)
       
        # Command heading (rotation/yaw)
        command_yaw(velocity[3], relative=False)   
        
        
        
    else:
        print("Code stalled. Waiting for safety switch to reactivate")
