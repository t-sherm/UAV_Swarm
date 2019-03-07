# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 18:49:20 2019

@author: UAV Lab Computer
Tristan Sherman

Cool stuff UAV Swarm Cal Poly Pomona 2019
"""
import ftl_definitions
import sys
import dronekit


# Startup
# Connect to vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)
# Send ack of successful connect
# ---------- TODO -----------------
# Tristan Cady: make a function that sends an ack

# Wait for toggle from aircraft
# Takeoff to 10m
safetySw = 0
while safetySw < 1500:
    
    safetySw = vehicle.channels['1']
    print (" Ch1: %s" % safetySw)
    

# Wait for receipt of messages
# ------------ TODO -------------
# Tristan Cady: Somehow have the code wait until we make sure the connection to the other computer is secured


# While loop ------------------------
while True:
    
    if safetySw > 1500:
        
        #Calculatie new position      
        # Get velocity and position of the controlled boid
        boid.velocity = vehicle.velocity
        boid.position = vehicle.global_relative_frame
            
        # Obtain the heading value of the controlled boid
        boid.hdg = vehicle.heading
        
    
        # ---------------------- TODO ------------------------
        # EVENTUALLY: Determine which boid is the local leader
        
        
        #----------------------- TODO --------------------------
        # TRISTAN CADY: Obtain position, velocity, and heading of other drone
        # Obtain position, velocity and heading of the other boid
        
        
        #Calculate steering vector for the boid
        cohesionVector = calculateCohesionVector(boid,j)
        separationVector = calculateSeparationVector(boid,j)
        velocity = updateVelocity(boid,1,cohesionVector, separationVector)
        
        # Command a velocity to the boid
        send_ned_velocity(vehicle, velocity[0], velocity[1], [2], 1)
       
        # Command heading (rotation/yaw)
        command_yaw(velocity[3], relative=False)   
        
        
        
    else:
        print("Code stalled. Waiting for safety switch to reactivate")