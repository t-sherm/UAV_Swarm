"""
Spyder Editor

Boids Algorithm With Added Behavior for UAV Swarm Mapping

This code connects with V-Rep to demonstrate the mapping of an area with boids algorithm

Author: Tristan Sherman
Email: Tristan.m.sherman@gmail.com
Date: 3/13/18


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
import random
import numpy as np

# This represents the mapped area of the ground
mapped = np.zeros(shape=(20,20))

#Do some more configuration for the boids algorithm
N = 10
boid = [1]*N
targetHandle = [1]*N
quadHandle = [1]*N

# 1 == yes, 0 == no
# Desired behaviors: Cohesion - Separation - Alignment - Boundary Avoidance - Eccentricity - Velocity Gradient
behaviors =       [     1     ,     1      ,     1     ,         1          ,      0      ,       1      ]
#how fast the position vectors update
timestep = .01

#Desired simulation run time (for development only)
simTime = 20 #seconds
cycles = simTime/timestep

#Additional configuration for boids algorithm
separationDistance = .5 #Distance boids must stay away from each other
maxVelocity = 1.5 * timestep # is actualfly the distance the boid will travel in one timestep
neighborDistance = 1.6 # how close the other boids need to be to be affected by other boids
maxSteering = 0.3 # for velocity alignment
maxViewingAngle = 80 #degrees
stickDistance = .5

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
      self.name = name
      self.count = 0
      Boid.empCount += 1
      Boid.stickTime = 0
      #Constants for vectors
      Boid.momentumConstant = 100
      Boid.cohesionConstant = .4
      Boid.separationConstant = .45
      Boid.velocityConstant = .2
      Boid.boundaryConstant = .4
      Boid.eccentricityConstant = 1
      Boid.eccentricityVector = [0,0,0]
      Boid.sameVelocity = 1
      Boid.gradientConstant = .1
      #a, b, c from distance equation
      Boid.boundaryStick = [0,0,0,0] #[If we are avoiding boundaries, a, b,c]

# Update mapped area
def mapArea(mapMatrix, boidVector, currentBoid):
    
    
    #Finds integers of position vectors
    x = math.floor(boidVector[currentBoid].position[0])
    y = math.floor(boidVector[currentBoid].position[1])
    
    #Accounts for translation from position to vector index
    if boidVector[currentBoid].position[0] > 4.5:
        x = 5
    if boidVector[currentBoid].position[1] > 4.5:
        y = 5
    if x <= -5:
        x = -4
    if y <= -5:
        y = -4
    
    #print('x=',x,'y=',y)
    #Fill in mapped sections
    for i in range(2):
        for j in range(2):
            #Basically fills the position with the corresponding index on the mapped matrix
            mapMatrix[(8+2*x)+i,(8+2*y)+j] = 1
    return mapMatrix
        
        
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

def velocityDirection(vector):
    angle = math.degrees(math.atan2(vector[1],vector[0]))
    return angle
    
    
# Updates the boid's velocity vector with factors included
def updateVelocity(boidVector,boid, numAhead, cohesion, separation,alignment, boundary, eccentricity, gradient):
    
    #Multiply vectors by constants
    momentum = [x*boidVector[boid].momentumConstant for x in boidVector[boid].velocity]
    cohesion = [x*boidVector[boid].cohesionConstant for x in cohesion]
    separation = [x*boidVector[boid].separationConstant for x in separation]
    alignment = [x*(boidVector[boid].velocityConstant ) for x in alignment]
    boundary = [x*boidVector[boid].boundaryConstant for x in boundary]
    eccentricity = [x*boidVector[boid].eccentricityConstant for x in eccentricity]
    #gradient = [x*boidVector[boid].gradientConstant for x in gradient]
    
    velocity = [x+y for x,y in zip(cohesion,separation)]
    velocity = [x+y for x,y in zip(velocity, momentum)]
    velocity = [x+y for x,y in zip(velocity, alignment)]
    velocity = [x+y for x,y in zip(velocity, boundary)]
    velocity = [x+y for x,y in zip(velocity, boidVector[boid].velocity)]
    velocity = [x+y for x,y in zip(velocity, eccentricity)]
    #velocity = [x+y for x,y in zip(velocity, gradient)]
    
    
    velocityVector = normalize(velocity)
    velocityVector = [x*(maxVelocity)*(1+numAhead/5) for x in velocityVector]
    #print(velocityVector)
    return velocityVector
    
    # Calculate the cohesion vector direction. Magnitude = max velocity
def calculateCohesionVector(boidVector, currentBoid):
    centerPosition = [0.0,0.0,0.0]
    count = 0
    #find average position of all boids
    for i in range(N):
        distance = distanceBetween(boidVector[i].position,boidVector[currentBoid].position)
        if distance[0] < neighborDistance:
            centerPosition[:] = [sum(x) for x in zip(boidVector[i].position,centerPosition)]
            count = count+1
            
    
    if count > 0:
        #divide numerator by number of boids to find average position
        centerPosition[:] = [x / count for x in centerPosition]
        #Find vector from boid to center of swarm
        distanceToCenter = distanceBetween(centerPosition, boidVector[currentBoid].position)
        
        # accounts for divzero errors
        if distanceToCenter[0] == 0:
            distanceToCenter[0] = 0.0001
        
        cohesionVector = [(x-y)/distanceToCenter[0] for x,y in zip(centerPosition, boidVector[currentBoid].position)]
        cohesionVector = [x*maxVelocity for x in cohesionVector] #Normalized vector times max velocity   
        return cohesionVector
    else: return boid[currentBoid].velocity
    
    
    # Separation vector points away from other boids. Magnitude increases as boids get closer
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
                
                if distance[0] == 0:
                    distance[0] = 0.0001
                    
                separationVector = [x/ (2*distance[0]) for x in separationVector]
                count = count+1
                
                
    if count > 0:
        #separationVector = normalize(separationVector)
        #separationVector = [x*maxVelocity for x in separationVector]
        return separationVector
    else: return boid[currentBoid].velocity
    
    
    # Aligns the boid velocity vector with the average velocity of neighbors
def calculateAlignmentVector(boidVector, currentBoid):
    count = 0
    alignmentVector = [0,0,0]
    
    for i in range(N):
        distance = distanceBetween(boidVector[i].position,boidVector[currentBoid].position)
        if distance[0] <= neighborDistance:
            alignmentVector = [sum(x) for x in zip(boid[i].velocity,alignmentVector)]
            count = count+1
            
    alignmentVector = [x/N for x in alignmentVector]
    
    if count >= 1:
        alignmentVector = normalize(alignmentVector)
        alignmentVector = [x*maxSteering for x in alignmentVector]
        return alignmentVector
    else: return boid[currentBoid].velocity
   
    
    
    
    
    
#Defines search area for boids
def calculateBoundaryVector(boidVector, currentBoid):
    boundaryVector = [0,0,0]
    dOther = 0
    neighbors = 0
    numStick = 0
    
    #print(boid[currentBoid].position[0])
    #Check to see if the boundary is hit (This is the boundary geometry information)

    if boid[currentBoid].position[0] > 5:
        boundaryVector = [-(boid[currentBoid].position[0]/10)**2,.1,0]
        boid[currentBoid].boundaryStick = [1,1,0,-5]
        
        
    if boid[currentBoid].position[0] < -5:
        boundaryVector = [(boid[currentBoid].position[0]/10)**2,-.1,0]
        boid[currentBoid].boundaryStick = [1,1,0,5]
        

    if boid[currentBoid].position[1] > 5:
        boundaryVector[1] = -(boid[currentBoid].position[1]/10)**2
        boundaryVector[0] = boundaryVector[0] - .1
        boid[currentBoid].boundaryStick = [1,0,1,-5]
        
        
    if boid[currentBoid].position[1] < -5:
        boundaryVector[1] = (boid[currentBoid].position[1]/10)**2
        boid[currentBoid].boundaryStick = [1,0,1,5]
        boundaryVector[0] = boundaryVector[0] + .1
        
        
    if boid[currentBoid].position[2] > 2:
        boundaryVector[2] = -(boid[currentBoid].position[2]/2)**2
    if boid[currentBoid].position[2] < .5:
        boundaryVector[2] = (boid[currentBoid].position[2]/1)**2
    #print(boundaryVector)
    
    #Check to see if any boid ahead of the current boid is sticking to the wall 
    #Calculate direction of current boid from velocity vector
    for i in range(N):
     
        distance = distanceBetween(boidVector[i].position,boidVector[currentBoid].position)
        if distance[0] <= neighborDistance and i !=currentBoid  :
            neighbors +=1
          
        #Check to see if any other boid is between the wall and the current boid
            a = boidVector[currentBoid].boundaryStick[1]
            b = boidVector[currentBoid].boundaryStick[2]
            c = boidVector[currentBoid].boundaryStick[3]
            if a !=0 or b !=0:
                #print('comparing positions with boid', i)
                dOther = abs(a*boidVector[i].position[0] + b*boidVector[i].position[1] + c)/math.sqrt(a**2+b**2)
                dCurrent = abs(a*boidVector[currentBoid].position[0] + b*boidVector[currentBoid].position[1] + c)/math.sqrt(a**2+b**2)
            
                
        if boidVector[i].boundaryStick[0] == 1 and dOther != 0:
            numStick += 1
            if dOther < dCurrent and numStick > 1:
                        
                boidVector[currentBoid].boundaryStick[0] = 0
                break
        
    if neighbors == 0:
        boidVector[currentBoid].boundaryStick[0] = 0
    #Find normal to vector
    
    a = boidVector[currentBoid].boundaryStick[1]
    b = boidVector[currentBoid].boundaryStick[2]
    c = boidVector[currentBoid].boundaryStick[3]
    if c < 0:
        normalVector = normalize([-a,-b,0])
    else:
        normalVector = normalize([a,b,0])
    #Distance from boid position to relevant wall
    #print(boidVector[currentBoid].boundaryStick[0])
    #print(normalVector)
    if boidVector[currentBoid].boundaryStick[0] == 1:
        dCurrent = abs(a*boidVector[currentBoid].position[0] + b*boidVector[currentBoid].position[1] + c)/math.sqrt(a**2+b**2)
        if dCurrent > stickDistance:
            stickVector = [x*(stickDistance-dCurrent) for x in normalVector]
            boundaryVector = [sum(x) for x in zip(stickVector,boundaryVector)]
            #print('Boid ', currentBoid, 'is ', dCurrent, 'away from the wall with a vector of ', boundaryVector )
    #if boid.boundaryStick[0] == 1:
    
    
    
       
    
    
    #print('')
    #print(boundaryVector)
    return boundaryVector






# Gives the tendency to separate from a group after a certain amount of time
def calculateEccentricityVector(boidVector, currentBoid):
    number = 0
    for i in range(N):
        #Count boids in neighbor distance
        distance = distanceBetween(boidVector[i].position,boidVector[currentBoid].position)
        if distance[0] < neighborDistance and i != currentBoid:
            number +=1
  
    # If there are any boids near the current one then keep counting        
    if number > 0:
        boidVector[currentBoid].count+= (timestep)
    
    if boidVector[currentBoid].count > 10 and boidVector[currentBoid].eccentricityVector == [0.0,0.0,0.0]:
        boidVector[currentBoid].eccentricityVector = [random.randint(-100,100), random.randint(-100,100), 0]
        boidVector[currentBoid].sameVelocity = 1
        
        
    if boidVector[currentBoid].count > 12 and boidVector[currentBoid].sameVelocity == 1:
        boidVector[currentBoid].eccentricityVector = [random.randint(-100,100), random.randint(-100,100), 0]
        boidVector[currentBoid].sameVelocity = 0
    
    if number == 0:
        boidVector[currentBoid].count = 0
        boidVector[currentBoid].eccentricityVector = [0,0,0]
        
    eccentricity = normalize(boidVector[currentBoid].eccentricityVector)
    eccentricity = [x*maxVelocity for x in boidVector[currentBoid].eccentricityVector]
    #print(eccentricity)
    return eccentricity

def calculateGradient(boidVector, currentBoid):
    angleSum = 0
    numAhead = 0
    #Calculate direction of current boid from velocity vector
    #print(boidVector[currentBoid].velocity)
    currentDirection = velocityDirection(boidVector[currentBoid].velocity)
    for i in range(N):
     
        #Calculate angle of the other boid with the current boid as the center of the coordinate system
        deltay = boidVector[i].position[1]-boidVector[currentBoid].position[1]
        deltax = boidVector[i].position[0]-boidVector[currentBoid].position[0] 
        otherDirection = math.degrees(math.atan2(deltay,deltax))
        
        angleBetween = (currentDirection - otherDirection)
        #print(angleBetween)
        distance = distanceBetween(boidVector[i].position,boidVector[currentBoid].position)
        if distance[0] <= neighborDistance and i !=currentBoid  :
           # print('Neighbor^^^^')
            
            #If the other boid is ahead of the current boid
            if abs(angleBetween) < maxViewingAngle:
                angleSum = angleSum + otherDirection
                
            #Tell how many are ahead
                numAhead +=1
            
    if numAhead > 0:
        avgAngle = angleSum/numAhead
        x = boidVector[currentBoid].velocity[1] + numAhead*math.cos(avgAngle)
        y = boidVector[currentBoid].velocity[1] + numAhead*math.sin(avgAngle)
        
    else:
        x = 0
        y = 0
    #print(numAhead)
    GradientVector = [x,y,0]
    return numAhead
    

        
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
    boid[0] = Boid(boidName,[-4.8,-5,1],[1,0,0])
    boid[1] = Boid(boidName,[-4.8,-4.3,1],[1,0,0])
    boid[2] = Boid(boidName,[-4.8,-3.4,1],[1,0,0])
    boid[3] = Boid(boidName,[-4.8,-5,1],[1,0,0])
    boid[4] = Boid(boidName,[-4.8,-4.3,1],[1,0,0])
    boid[5] = Boid(boidName,[-4.8,-3.7,1],[1,0,0]) 
    boid[6] = Boid(boidName,[-4.8,-5,1],[1,0,0])
    boid[7] = Boid(boidName,[-4.8,-4.3,1],[1,0,0])
    boid[8] = Boid(boidName,[-4.8,-3.7,1],[1,0,0]) 
    boid[9] = Boid(boidName,[-4.8,-4.5,1],[1,0,0])
    
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

# These are zero if they are not used
numAhead = 0
cohesionVector = [0,0,0]
separationVector = [0,0,0]
alignmentVector = [0,0,0]
boundaryVector = [0,0,0]
eccentricityVector = [0,0,0]
count = 0
systemTime = 0
gradientVector = [0,0,0]
M = 0
while 0 <1:
    
    if systemTime > .3:
        M = 3
    if systemTime > 1:
        M = 6
    if systemTime > 1.5:
        M = 10
    
    #Calculatie new positions except for mothership
    for j in range(M):
            
        #Calculate steering vectors for each boid   
        if behaviors[0] == 1:
            cohesionVector = calculateCohesionVector(boid,j)
        if behaviors[1] == 1:
            separationVector = calculateSeparationVector(boid,j)
        if behaviors[2] == 1:
            alignmentVector = calculateAlignmentVector(boid, j)
        if behaviors[3] == 1:
            boundaryVector = calculateBoundaryVector(boid,j)
        if behaviors[4] == 1:
            eccentricityVector = calculateEccentricityVector(boid, j)
        if behaviors[5] == 1:
            numAhead = calculateGradient(boid, j)
            #gradientVector = gradientOut[0]
            #numAhead = gradientOut[0]
            #print(numAhead)
            
        #print(j)
        #print(' ')
       
        # Calculate steering velocity from steering vectors
        boid[j].velocity = updateVelocity(boid,j, numAhead, cohesionVector, separationVector, alignmentVector, boundaryVector, eccentricityVector, gradientVector)       
        boid[j].position = [sum(x) for x in zip(boid[j].position, boid[j].velocity)]
                
        #send boid
        #print([boid[j].position])
        returnCode = vrep.simxSetObjectPosition(clientID,targetHandle[j],-1,boid[j].position,vrep.simx_opmode_oneshot)
        if boundaryVector == [0,0,0]:
            mapped = mapArea(mapped, boid,j)
    # If every value in the matrix is 0, the whole thing is mapped
    if np.count_nonzero(mapped) == 400:
        #count = count + 1
        print(systemTime)
        time.sleep(10)
        mapped = np.zeros(shape=(20,20))
        
       
    #print(numAhead)
    print(mapped)
    print('')
    systemTime = systemTime + timestep
    time.sleep(timestep)



'''
    #Check to see if any boid ahead of the current boid is sticking to the wall 
    #Calculate direction of current boid from velocity vector
    currentDirection = velocityDirection(boidVector[currentBoid].velocity)
    for i in range(N):
     
        distance = distanceBetween(boidVector[i].position,boidVector[currentBoid].position)
        if distance[0] <= neighborDistance and i !=currentBoid  :
            
            #Calculate angle of the other boid with the current boid as the center of the coordinate system
            deltay = boidVector[i].position[1]-boidVector[currentBoid].position[1]
            deltax = boidVector[i].position[0]-boidVector[currentBoid].position[0] 
            otherDirection = math.degrees(math.atan2(deltay,deltax))
            angleBetween = (currentDirection - otherDirection)
            
            #If the other boid is ahead of the current boid
            if abs(angleBetween) < maxViewingAngle:
                
                #Check to see if any other boid is between the wall and the current boid
                a = boidVector[currentBoid].boundaryStick[1]
                b = boidVector[currentBoid].boundaryStick[2]
                c = boidVector[currentBoid].boundaryStick[3]
                if a !=0 or b !=0:
                    #print('comparing positions with boid', i)
                    dOther = abs(a*boidVector[i].position[0] + b*boidVector[i].position[1] + c)/math.sqrt(a**2+b**2)
                    dCurrent = abs(a*boidVector[currentBoid].position[0] + b*boidVector[currentBoid].position[1] + c)/math.sqrt(a**2+b**2)
                
                #If there is a boid in front of this one doing a boundary stick then exit the loop
                if boidVector[i].boundaryStick[0] == 1:
                    if dOther < dCurrent:
                        boidAheadStick = 1
                        break
                    
    if boidAheadStick == 1:
        boidVector[currentBoid].boundaryStick[0] = 0
        dCurrent = 0
        
    else:
        boidVector[currentBoid].boundaryStick[0] = 1
        print('boid',currentBoid, 'is doing boundary stick')
        a = boidVector[currentBoid].boundaryStick[1]
        b = boidVector[currentBoid].boundaryStick[2]
        c = boidVector[currentBoid].boundaryStick[3]
        if a != 0 or b !=0:
            dCurrent = abs(a*boidVector[currentBoid].position[0] + b*boidVector[currentBoid].position[1] + c)/math.sqrt(a**2+b**2)
            
        
    
                   
    if boidVector[currentBoid].boundaryStick[0] == 1 and dCurrent > stickDistance:
        boundaryVector = [x*(stickDistance-dCurrent)**2 for x in normalVector]
        print(boundaryVector)
    '''