"""
Created on Tue Feb 26 19:36:29 2019

Follow the leader dronekit implementation
Helper functions

@author: UAV Lab Computer
Tristan Sherman
"""

import math
from pymavlink import mavutil
pi = 3.1415

#Set up boid class    
class Boid:
   'Boid class'
   empCount = 0
   
   def __init__(self, name, position, velocity):
      self.position = position
      self.velocity = velocity
      Boid.hdg = 0
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

# Controllable variables

phantomDistance = 10 # meters. Distance behind 
Vmax = 1.5 # meters/second

# If the drone is within this distance of it's commanded position it will start slowing down
relaxDistance = 4 # meters

# Distance from the other drone that it is allowed to be
separationDistance = 5 # meters

#Magnitude of vector
def mag(vector):
    square = [abs(x**2) for x in vector]
    sumInside = sum(square)
    magnitude = [ math.sqrt(sumInside) ]
    return magnitude

# Vector normalization
def norm(vector):
    square = [abs(x**2) for x in vector]
    sumInside = sum(square)
    magnitude = [ math.sqrt(sumInside) ]
    if magnitude[0] > 0:
        unitVector = [x/magnitude[0] for x in vector]
    else: unitVector = [0,0,0]
    return unitVector


def GPStoNED(location1, location2):
    #Vector from currentPos to nextPos

    #TO_RADIANS = pi/180
   # KM_TO_M = 1000
   # RADIUS_EARTH = 6378.037 # km
    
    location1Lat = location1[0]
    location1Lon = location1[1]
    location2Lat = location2[0]
    location2Lon = location2[1]
    
# =============================================================================
#      #Convert to radians
#     latRad = currentLat * TO_RADIANS
#     lonRad = currentLon * TO_RADIANS
#     
#     #Find delta in positions
#     deltaLat = (nextLat - currentLat) * TO_RADIANS
#     deltaLon = (nextLon - currentLon) * TO_RADIANS
#     
#     # Math from: http://www.movable-type.co.uk/scripts/latlong.html
#     # Using the above math to calculate dlon, we assume dlat = 0. This
#     # simplifies the equation
#     
#     # Calculate x cartesian distance (longitude distance): no longer NED frame
#     varDlon = (math.cos(latRad))**2 * (math.sin(deltaLon/2))**2
#     dx_cart = 2 * math.atan2(math.sqrt(varDlon),math.sqrt(1-varDlon)) * RADIUS_EARTH * KM_TO_M
#     
#     if deltaLon < 0:
#         dx_cart = -dx_cart
#     
#         
#     # Calculate dlat. Assume dlon = 0
#     varDlat = math.sin(deltaLat/2)**2
#     dy_cart = 2 * math.atan2(math.sqrt(varDlat),math.sqrt(1-varDlat)) * RADIUS_EARTH * KM_TO_M
#     
#     if deltaLat < 0:
#         dy_cart = -dy_cart
#     
#     # Convert to NED frame
#     dx_ned = dy_cart
# =============================================================================
#    dy_ned = dx_cart
        
# def get_distance_metres(aLocation1, aLocation2): Function pulled from dronekit documentation
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = location2Lat - location1Lat
    dlong = location2Lon - location1Lon
    #return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    return [dlat,dlong]

    # Definitions TODO: Try to use a pointer instead of direct substitution?
def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    
    vehicle.send_mavlink(msg)
    
    
    
    
    
    #This function will attract the uav to a phantom boid behind its lead drone
    #This phantom drone gives the position this drone SHOULD be at
    #It is just a distance d directly behind the lead drone in line with it's heading
    
def calculateCohesionVector(boidVector, currentBoid):
    # INPUT: pos, hdg in global coordinates
    stop = 0

    
    leaderPosition = boidVector[currentBoid+1].position #Start with lead drone position
    leaderHdg = boidVector[currentBoid+1].hdg # Heading of leader
    currentPosition = boidVector[currentBoid].position # Position of the controlled drone
    currentHdg = boidVector[currentBoid].hdg # heading of controlled drone
    
    
    #Position of the phantom drone (a certain distance behind the leader in NED)
    #                            Latitude (x - North)               Longitude (y - East)
    phantomDisplacement = [math.cos(leaderHdg)*-phantomDistance,  math.sin(leaderHdg)*-phantomDistance,0]
    
    # Get the cartesian distance between drone and leader
    R_current_to_leader = GPStoNED(currentPosition, leaderPosition)
    
    # Total vector addition to get distance from current position to phantom    
    R_phantom = [x+y for x,y in zip(R_current_to_leader, phantomDisplacement)]
  
    R_phantom_mag = mag(R_phantom)  # Magnitude of vector
    
    # Angle to phantom drone atan2 = (longitude - East (y)/ latitude - North (x))
    theta_phantom  = math.atan2(R_phantom[0], R_phantom[1])

    # Maybe don't need this
    if R_phantom_mag[0] < .01:
        stop = 1;
   
        
    # Non dimensionalized velocity magnitude using plot of inverse tangent
    cohesionMag = 2/pi * math.atan(R_phantom_mag[0] * relaxDistance/6)
    
    # Vector pointing towards the phantom vector with magnitude relative to V_max
    # [ Vx, Vy, Vz, New Heading angle]
    cohesionVector = [cohesionMag * math.cos(theta_phantom), cohesionMag * math.sin(theta_phantom), 0, leaderHdg]
    #cohesionVector = [0, 0, 0, 0]
    return cohesionVector





def calculateSeparationVector(boidVector, currentBoid):
    
    leaderPosition = boidVector[currentBoid+1].position #Start with lead drone position
    leaderHdg = boidVector[currentBoid+1].hdg # Heading of leader
    currentPosition = boidVector[currentBoid].position # Position of the controlled drone
    currentHdg = boidVector[currentBoid].hdg # heading of controlled drone
    
    R_to_leader = GPStoNED(currentPosition, leaderPosition)
    R_mag = mag(R_to_leader)
    print(R_mag)
    if R_mag[0] <= separationDistance:
        
        # Starting at 20% of maximum velocity
        separationMagnitude = 0.2 * 1/(R_mag[0]/separationDistance)
        
        #Separation vector = direction of (currentPosition - other boid position) * magnitude
        separationVector = [x/-R_mag[0] for x in R_to_leader]
        separationVector = [x * separationMagnitude for x in separationVector]
        
                            #    Vx                       Vy        alt  hdg
        separationVector = [separationVector[0], separationVector[1], 0,  0  ]
        #separationVector = [0,0,0,0]       
        return separationVector
    return [0,0,0,0]
    
    
    # Updates the boid's velocity vector with factors included
def updateVelocity(boidVector,boid,cohesion, separation):
    
    # Velocity = cohesion + separation vectors * Vmax
    velocity = [x+y for x,y in zip(cohesion, separation)]
    velocity = [x * Vmax for x in velocity]
    velocity[3] = cohesion[3] # heading
    
    return velocity


def command_yaw(vehicle, heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
