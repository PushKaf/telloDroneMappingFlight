import keyboardControlTelloModule as km
from djitellopy import Tello
from time import sleep
import numpy as np
import cv2
import math

###############################Parameters###################################
fSpeed = 117/10 #foreward speed in cm/s 117 cm in 10 secs (15cm/s)
aSpeed = 360/10 #angular speed in cm/s 360 degs in 10 secs (50cm/s)
interval = 0.25

dInterval = fSpeed * interval #gives distance everytime we move 1 unit
aInterval = aSpeed * interval #Gives angle every time we move 1 unit
############Dont Change Unless You Kinda Know What You're Doing.############

x,y = 500,500
angle = 0 #angle
yaw = 0
km.init()
drone = Tello()
drone.connect()
print(drone.get_battery())
points = [(0, 0), (0, 0)]

#getting keyinp using the provided module
def getKeyInput():
    leftRight, foreBack, upNDown, yawVel = 0, 0, 0, 0
    
    speed = 15
    aSpeed = 50
    global x, y, angle, yaw
    distance = 0 #distance
     
    #Controls with WASD;
    if km.getKey("w"): 
        foreBack = speed #Foreward
        distance = dInterval
        angle = 270
    elif km.getKey("s"): 
        foreBack = -speed #Back
        distance = -dInterval
        angle = -90 
    if km.getKey("a"): 
        leftRight = -speed #Left
        distance = dInterval
        angle = -180
    
    elif km.getKey("d"): 
        leftRight = speed #Right
        distance = -dInterval
        angle = 180
    
    if km.getKey("UP"): 
        upNDown = speed #Makes the drone go up
    elif km.getKey("DOWN"): 
        upNDown = -speed #Makes the drone go down
    if km.getKey("RIGHT"): 
        yawVel = -aSpeed #Turns the drone counterclockwise
        yaw += aInterval

    elif km.getKey("LEFT"): 
        yawVel = aSpeed #Turns the drone clockwise
        yaw -= aInterval

    #Landing and Takeoff
    elif km.getKey("ESCAPE"): 
        drone.emergency() #EMERGENCY LANDING; if land command isnt going through
    elif km.getKey("SPACE"): 
        drone.takeoff() #Takeoff the drone
    elif km.getKey("RETURN"): 
        drone.land() #Enter Key

    sleep(interval)

    angle += yaw
    x += int(distance*math.cos(math.radians(angle)))
    y += int(distance*math.sin(math.radians(angle)))

    return [leftRight, foreBack, upNDown, yawVel, x, y]

#draws the points in the cv2 screen
def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 7, (255, 0, 0), cv2.FILLED)
    cv2.circle(img, points[-1], 8, (0, 255, 0), cv2.FILLED)
    cv2.putText(img, f"({(points[-1][0]-500)/ 100}, {(points[-1][1]-500)/ 100}m)", (points[-1][0]+10, points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

#camera features
drone.streamoff()
drone.streamon()

while True:
    val = getKeyInput()
    #drone movements
    drone.send_rc_control(val[0], val[1], val[2], val[3])
    
    img = np.zeros((1000, 1000, 3), np.uint8)
    if(points[-1][0] != val[4] or points[-1][1] != val[5]):
        points.append((val[4], val[5]))
    
    #drawing the poings with previous data passed
    drawPoints(img, points)

    #getting and resizing the drone fottage
    droneFootage = drone.get_frame_read().frame
    droneFootage = cv2.resize(droneFootage, (500,500))

    #showing the mapping of the drone and the drone footage
    cv2.imshow("Drone Footage", droneFootage)
    cv2.imshow("Out", img)
    cv2.waitKey(1)


