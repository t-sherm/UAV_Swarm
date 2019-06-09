# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.

# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
"""

#Set up variables
import vrep
import sys
import time
import math
#import random
import numpy as np

mapped = np.ones(shape=(20,20))


#Do some more configuration for the boids algorithm
N = 10
boid = [1]*N
targetHandle = [1]*N
quadHandle = [1]*N

#how fast the position vectors update
timestep = .01

#Desired simulation run time
simTime = 20 #seconds

cycles = simTime/timestep


separationDistance = .5 #Distance boids must stay away from each other
maxVelocity = 3 * timestep # is actually the max distance the boid will travel in one timestep
neighborDistance = 1
maxSteering = 0.3
phantomDistance = separationDistance + .2

#Set up for vrep communication
targetLabel = "Quadricopter_target#"
quadLabel = "Quadricopter#"

    

#Set up boid class    
class Boid:
   'Boid class'
   empCount = 0
   
   def __init__(self, name, position, velocity):
      self.position = position
      self.velocity = velocity
      self.hdg = 0 #heading of drone. 0 degrees north for now
      self.name = name
      Boid.empCount += 1
      #Constants for vectors
      Boid.cohesionConstant = .4
      Boid.separationConstant = .3

        
        
#Distance function
def distanceBetween(a,b):
    #Difference
    Difference = [x-y for x,y in zip(a,b)]
    #square of difference
    square = [x**2 for x in Difference]
    #Sum of squares
    sumInside = sum(square)
    return [ math.sqrt(sumInside) ]
    
# Calculate vector magnitude    
def magnitude(vector):
    square = [x**2 for x in vector]
    sumInside = sum(square)
    return [ math.sqrt(sumInside) ]
    
# Vector normalization
def normalize(vector):
    square = [abs(x**2) for x in vector]
    sumInside = sum(square)
    magnitude = [ math.sqrt(sumInside) ]
    if magnitude[0] > 0:
        unitVector = [x/magnitude[0] for x in vector]
    else: unitVector = [0,0,0]
    return unitVector

#Calculate heading from velocity
def headingCalc(velocity):
    x = velocity[0]
    y = velocity[1]
    
    if x == 0 and y > 0:
        hdg = 0
        return hdg
    elif x == 0 and y < 0:
        hdg = math.pi
        return hdg
    elif x > 0 and y == 0:
        hdg = math.pi/2
        return hdg
    elif x < 0 and y == 0:
        hdg = 3*math.pi
        return hdg
    elif x == 0 and y == 0:
        return 0
    inside = x/y #x/y so North (up) is 0 degrees
    hdgTemp = math.atan(inside)
    # South-East quadrant
    if x > 0 and y < 0:
        hdg = math.pi + hdgTemp
    elif x < 0 and y < 0:
        hdg = math.pi + hdgTemp
    elif x < 0 and y > 0:
        hdg = math.pi*2 + hdgTemp
    else:
        hdg = hdgTemp
    return hdg
    

# Updates the boid's velocity vector with factors included
def updateVelocity(boidVector,boid,cohesion, separation):
    
    
    #Get cohesion vector
    if cohesion[1] == 0:
        cohesion = [x*boidVector[boid].cohesionConstant for x in cohesion[0]]
    else:
        cohesion = [0,0,0]
        
    print("cohesion vector")
    print(cohesion)
    
    separation = [x*boidVector[boid].separationConstant for x in separation]
    print("separation vector")
    print(separation)
    velocity = [x+y for x,y in zip(cohesion,separation)]
    #velocity = [x+y for x,y in zip(velocity, boidVector[boid].velocity)]
    #print(velocity)
    
    square = [abs(x**2) for x in velocity]
    sumInside = sum(square)
    magnitude = [ math.sqrt(sumInside) ]
    
    if magnitude[0] > maxVelocity:
        velocityVector = normalize(velocity)
       # print(velocityVector)
        velocityVector = [x*maxVelocity for x in velocityVector]
    else:
        velocityVector = velocity
    velocityVector[2] = 0
    return velocityVector
    

    #This function will attract the uav to a phantom boid behind its lead drone
    #This phantom drone gives the position this drone SHOULD be at
    #It is just a distance d directly behind the lead drone's heading
def calculateCohesionVector(boidVector, currentBoid):
#    print("start function")
    stop = 0
    cohesionVector = [0,0,0]
    hdg = boidVector[currentBoid-1].hdg
    
    #find the position of the phantom drone
    leaderPosition = boidVector[currentBoid-1].position #Start with lead drone heading
#    print("leadPosition")
#    print(leaderPosition)
    phantomDisplacement = [math.sin(hdg)*-phantomDistance,math.cos(hdg)*-phantomDistance,0]
    phantomPosition = [x+y for x,y in zip(leaderPosition, phantomDisplacement)]
#    print("phantomPosition")
#    print(phantomPosition)
    #Find vector from boid phantom drone
    distanceToPhantom = distanceBetween(phantomPosition, boidVector[currentBoid].position)
    if distanceToPhantom[0] < .01:
        stop = 1;
    else:
        cohesionVector = [(x-y)*math.sqrt(distanceToPhantom[0]) for x,y in zip(phantomPosition, boidVector[currentBoid].position)]

#    print([cohesionVector,stop])
#    print("end function")
    return [cohesionVector, stop]
    
    
def calculateSeparationVector(boidVector, currentBoid):
    count = 0
    separationVector = [0,0,0]
    for i in range(N):
        if currentBoid!=i:
            ##find distance between two boids. boid J is the current position
            distance = distanceBetween(boid[i].position,boid[currentBoid].position)
            if distance[0] <= separationDistance:
                #Separation vector = separationVector + currentPosition - other boid position
                separationVector = [x+y for x,y in zip(boid[currentBoid].position, separationVector)]
                separationVector = [x-y for x,y in zip(separationVector, boid[i].position)]
                
                if distance[0] < 0.0001:
                    distance[0] = 0.0001
                    
                separationVector = [x/ (distance[0]) for x in separationVector]
                count = count+1
                
                
    if count > 0:
        return separationVector
    else: return boid[currentBoid].velocity
    


 ## ---------------------   START OF ACTUAL CODE -----------------------------
#Connect to V-rep
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')
    
else:
    print( "Connection not successful")
    sys.exit('Could not connect')
    
    
#Initiate positions
for i in range(N):
    
    targetName = targetLabel+str(i)
    boidName = quadLabel+str(i)
    boid[i] = Boid(boidName,[0,0,1],[1,0,0])
    
    print(boidName)
    #grab handler for object we are controlling    
    errorCode, targetHandle[i]=vrep.simxGetObjectHandle(clientID,targetName, vrep.simx_opmode_blocking)
    #errorCode, quadHandle[i]=vrep.simxGetObjectHandle(clientID,boidName, vrep.simx_opmode_blocking)
    
    #set target parent as quad copter
    #vrep.simxSetObjectParent(clientID, targetHandle[i], quadHandle[i],0,vrep.simx_opmode_blocking)
    
    #Set initial position
    if i < N:
        returnCode = vrep.simxSetObjectPosition(clientID,targetHandle[i],-1,boid[i].position,vrep.simx_opmode_oneshot)
   
centerPosition = [0.0,0.0,0.0]
    
time.sleep(3)


# Run boid simulation ----------------------------------------------------------------------

count = 0
systemTime = 0
while count < 1:
        
    #Calculatie new positions
    leaderPosition = vrep.simxGetObjectPosition(clientID,targetHandle[0],-1,vrep.simx_opmode_oneshot)
    
    boid[0].velocity = [x-y for x,y in zip(leaderPosition[1], boid[0].position)]
    boid[0].position = leaderPosition[1]
    newHdg = headingCalc(boid[0].velocity)
    if newHdg != 0:
        boid[0].hdg = newHdg
    #print(boid[0].hdg)
    for j in range(N):
        if j > 0:
            
            #Calculate steering vectors for each boid   
            cohesionVector = calculateCohesionVector(boid,j)
            separationVector = calculateSeparationVector(boid,j)
            
            #print(boid[j].velocity)
            
            # Calculate steering velocity from steering vectors
            boid[j].velocity = updateVelocity(boid,j, cohesionVector, separationVector)       
            boid[j].position = [sum(x) for x in zip(boid[j].position, boid[j].velocity)]
            newHdg = headingCalc(boid[j].velocity)
            if newHdg != 0:
                boid[j].hdg = newHdg
            #send boid
            #print([boid[j].position])
            returnCode = vrep.simxSetObjectPosition(clientID,targetHandle[j],-1,boid[j].position,vrep.simx_opmode_oneshot)
            
    time.sleep(timestep)
print(systemTime)