from PIL import Image
import numpy as np
from numpy.lib.function_base import _diff_dispatcher
import math

k_searchInterval = 2 # the spacing between evaluated pixels
k_lineThickness = 10 # the line thickness drawn on the image
k_markerRadius = 2 # the radius for the marker on the output image
k_markerColor = [255, 40, 5] #Ferrari Red
k_orginColor = [0, 255, 0] # green
k_waypointStringingRadius = 23 # the maximum distance to look for a waypoint when creating a path
k_pixelScalingFactor = .009208 # how many meters are represented by the length 1 pixel
k_maxPathSmoothingDistance = 40 # how far between points the smoothing algorithim can go
k_convertPathToCode = True # wether or not to save the path as lines of code

baseImage = Image.open('BaseImage.jpg')
baseImageBitmap = np.array(baseImage) #3d image array [height, width, RGBA]
outputImageBitmap = baseImageBitmap # save for later use

baseHeight = baseImageBitmap.shape[0]
baseWidth = baseImageBitmap.shape[1]

newImage = Image.open('NewImage5.jpg')
newImageBitmap = np.array(newImage)

newHeight = newImageBitmap.shape[0]
newWidth = newImageBitmap.shape[1]

if(newHeight != baseHeight or newWidth != baseWidth):
    print("The base image and new image do not match size!")
    print(baseImageBitmap.shape)
    print(newImageBitmap.shape)

differencesArray = []
fullMatDifferenceArray = np.zeros((newHeight, newWidth))

print("Detecting changes in image")

heightCounter = 0
widthCounter = 0
while(heightCounter < newHeight): # go through each pixel and see if it has been signifigantly changed from the orignal image 
    while(widthCounter < newWidth):
        if(not np.allclose(newImageBitmap[heightCounter][widthCounter], baseImageBitmap[heightCounter][widthCounter], 0, 30)):
            differencesArray.append([heightCounter, widthCounter]) #record in an array of changed pixels
            fullMatDifferenceArray[heightCounter][widthCounter] = 1 #makes mat image of changed pixels
            #print(newImageBitmap[heightCounter][widthCounter], baseImageBitmap[heightCounter][widthCounter])
        widthCounter = widthCounter + k_searchInterval

    widthCounter = 0
    heightCounter = heightCounter + k_searchInterval

print("Searching for waypoints")

waypointArray = [] # the array of places where a waypoint should go
minimumWaypointPopulation = k_lineThickness * k_lineThickness / k_searchInterval / k_searchInterval * 0.785 * 0.85 #creates a minimum to place a marker based off of the searchInterval and the circularity constant 

for pixel in differencesArray: # goes through the changed pixels to find waypoints
    pixelX = pixel[1] 
    pixelY = pixel[0]

    if(fullMatDifferenceArray[pixelY][pixelX] == 1): # only evaluate pixel if in array -- doesnt bother with pixels that already have been accounted for
        waypointZone = [[max(pixelY - k_lineThickness / 2, 0), max(pixelX - k_lineThickness / 2, 0)], [min(pixelY + k_lineThickness / 2, newHeight), min(pixelX + k_lineThickness / 2, newWidth)]] 
        # the zone to check for the population of changed points

        populationCounter = 0 # the popluation of changed points in the wayPoint zone

        waypointZoneYCounter = waypointZone[0][0]
        while(waypointZoneYCounter < waypointZone[1][0]):
            waypointZoneXCounter = waypointZone[0][1]

            while(waypointZoneXCounter < waypointZone[1][1]):
                
                if(fullMatDifferenceArray[int(waypointZoneYCounter)][int(waypointZoneXCounter)] == 1):
                    populationCounter = populationCounter + 1
                waypointZoneXCounter = waypointZoneXCounter + 1

            waypointZoneYCounter = waypointZoneYCounter + 1

        if(populationCounter > minimumWaypointPopulation): # if the pixel is a valid waypoint add it 
            waypointArray.append(pixel)

            waypointZoneYCounter = waypointZone[0][0]
            while(waypointZoneYCounter < waypointZone[1][0]): # clears out the area cover by the waypoint
                waypointZoneXCounter = waypointZone[0][1]

                while(waypointZoneXCounter < waypointZone[1][1]): 
                    fullMatDifferenceArray[int(waypointZoneYCounter)][int(waypointZoneXCounter)] = 0
                    waypointZoneXCounter = waypointZoneXCounter + 1

                waypointZoneYCounter = waypointZoneYCounter + 1

print("Finding orgin")

orgin = [] # the orgin of the path
minGreenError = 1000
for waypoint in waypointArray: # finds the orgin marker in green
    waypointRGB = newImageBitmap[waypoint[0]][waypoint[1]]

    greenError = 0.0 # the error from the drakest green color
    orginErrorCounter = 0
    while(orginErrorCounter < 3):
        greenError = greenError + abs(waypointRGB[orginErrorCounter] - k_orginColor[orginErrorCounter])
        orginErrorCounter = orginErrorCounter + 1
        
    if(greenError < minGreenError): # if the waypoint is closer to true green than the current
        orgin = waypoint
        minGreenError = greenError

print("Stringing together path")

waypointMaskArray = np.zeros((newHeight, newWidth))
for waypoint in waypointArray: # goes through and marks the waypoints on an array
    waypointMaskArray[waypoint[0]][waypoint[1]] = 1
waypointMaskArray[orgin[0]][orgin[1]] = 0 # remove orgin as valid point

pathFound = True # changes to false if no valid link is found between two waypoints
previousPoint = [-1] # starts at -1 which is an invalid point -- 
startPoint = orgin # the point to start the path from
pathArray = [orgin] # the waypoints in order of the path 
while(pathFound):
    pathSearchZone = [[max(startPoint[0] - k_waypointStringingRadius, 0), max(startPoint[1] - k_waypointStringingRadius, 0)], [min(startPoint[0] + k_waypointStringingRadius, newHeight), min(startPoint[1] + k_waypointStringingRadius, newWidth)]] 
    waypointsInZoneArray = []

    pathSearchZoneYCounter = pathSearchZone[0][0]
    while(pathSearchZoneYCounter < pathSearchZone[1][0]): # goes through and finds all the other waypoints in the zone
        pathSearchZoneXCounter = pathSearchZone[0][1]

        while(pathSearchZoneXCounter < pathSearchZone[1][1]): 
            if(waypointMaskArray[pathSearchZoneYCounter][pathSearchZoneXCounter] == 1):
                waypointsInZoneArray.append([pathSearchZoneYCounter, pathSearchZoneXCounter])
            pathSearchZoneXCounter = pathSearchZoneXCounter + 1

        pathSearchZoneYCounter = pathSearchZoneYCounter + 1

    if(waypointsInZoneArray.__len__() == 0): # if there is no next waypoint in the search zone
        print("Path Terminated No Point Found")
        pathFound = False
    elif(waypointsInZoneArray.__len__() == 1): # if one go there
        previousPoint = startPoint
        startPoint = waypointsInZoneArray[0]
        pathArray.append(startPoint) # adds the new point to the path
        waypointMaskArray[startPoint[0]][startPoint[1]] = 0 # remove the waypoint from the array of valid points to go to
    elif(previousPoint[0] == -1): # if the first connection - go to the closest
        closestPoint = [0, 0, 100000] # the closest point to the orgin - [Y, X, Distance]
        
        for waypoint in waypointsInZoneArray:
            distance = math.sqrt(math.pow(startPoint[0] - waypoint[0], 2) + math.pow(startPoint[1] - waypoint[1], 2)) # distance from the previous point
            
            if(closestPoint[2] > distance):# if the waypoint is closest -- update the closest point
                closestPoint[2] = distance
                closestPoint[0] = waypoint[0]
                closestPoint[1] = waypoint[1]

        previousPoint = startPoint # remove the waypoint from the array of valid points to go to
        startPoint = [closestPoint[0], closestPoint[1]] # if the first connection - go to the closest
        pathArray.append(startPoint) # adds the new point to the path
        waypointMaskArray[startPoint[0]][startPoint[1]] = 0 # the closest point to the orgin - [Y, X, Distance]
    else: # if multiple points - use unlinearity and distance to calculate the next point
        previousAngle = math.pi / 2
        if(startPoint[1] != previousPoint[1]): # stops a division by zero
            previousAngle = math.atan((startPoint[0] - previousPoint[0]) / (startPoint[1] - previousPoint[1]))
        
        if(startPoint[1] - previousPoint[1] < 0):
            previousAngle = previousAngle + math.pi

        lowestUnlinearity = -1
        for waypoint in waypointsInZoneArray: # goes through and determine the lowest unlineraity
            currentAngle = math.pi / 2
            if(waypoint[1] != startPoint[1]): # stops a division by zero
                currentAngle = math.atan((waypoint[0] - startPoint[0]) / (waypoint[1] - startPoint[1]))
        
            if(waypoint[1] - startPoint[1] < 0):
                currentAngle = currentAngle + math.pi

            unlineraity = abs(currentAngle - previousAngle)
            if(unlineraity > math.pi):
                unlineraity = 6.14 - unlineraity

            if(unlineraity < lowestUnlinearity or lowestUnlinearity == -1):
                lowestUnlinearity = unlineraity
        
        unlineraityMax = lowestUnlinearity + math.pi / 6 # waypoints that are valid must be in this range + or -
        validWaypoints = [] # all of the points that are within the unlinearity max
        for waypoint in waypointsInZoneArray:
            currentAngle = math.pi / 2
            if(waypoint[1] != startPoint[1]): # stops a division by zero
                currentAngle = math.atan((waypoint[0] - startPoint[0]) / (waypoint[1] - startPoint[1]))
            
            if(waypoint[1] - startPoint[1] < 0):
                currentAngle = currentAngle + math.pi

            unlineraity = abs(currentAngle - previousAngle)
            if(unlineraity > math.pi):
                unlineraity = 6.14 - unlineraity

            if(unlineraity < unlineraityMax):
                validWaypoints.append(waypoint)

        closestPoint = [0, 0, 100000] # the closest point to the orgin - [Y, X, Distance]
        
        for waypoint in validWaypoints:
            distance = math.sqrt((math.pow(startPoint[0] - waypoint[0], 2) + math.pow(startPoint[1] - waypoint[1], 2))) # distance from the previous point
            
            if(closestPoint[2] > distance):# if the waypoint is closest -- update the closest point
                closestPoint[2] = distance
                closestPoint[0] = waypoint[0]
                closestPoint[1] = waypoint[1]

        previousPoint = startPoint # makes the start point into the next previous point
        startPoint = [closestPoint[0], closestPoint[1]] # new start point
        pathArray.append(startPoint) # adds the new point to the path
        waypointMaskArray[startPoint[0]][startPoint[1]] = 0 # remove the waypoint from the array of valid points to go to

'''
print("Smoothing Path")
waypointCounter = 0
while(waypointCounter < pathArray.__len__()):
    smoothPoint = True
    while(smoothPoint == True):
        point = pathArray[waypointCounter] # the base point

        # invalid points have a third entry which is -1
        nextPointCounter = 1
        pointFound = False # has a new point been found
        while(not pointFound): # while a valid next point has not been found
            if(pathArray[waypointCounter + nextPointCounter].__len__() != 2): # if the point is a vaild point
                invalidPoint = pathArray[waypointCounter + nextPointCounter]
                
                distance = math.sqrt(math.pow(point[0] - invalidPoint[0], 2) + math.pow(point[1] - invalidPoint[1], 2))
                if(distance < k_maxPathSmoothingDistance): # if the point is within the smoothing distance continue
                    nextPointCounter = nextPointCounter + 1
                else:
                    pointFound = True # marks the point as found but do not smooth becuase the point is invalid
                    smoothPoint = False
            else: 
                pointFound = True

        if(smoothPoint): # if the point is to be attempted to be smooth
            nextPoint = pathArray[waypointCounter + nextPointCounter]
            nextNextPoint = pathArray[waypointCounter + nextPointCounter + 1]
            nextNextNextPoint = pathArray[waypointCounter + nextPointCounter + 2]
'''



print("Creating ouput image")

previousPoint = [-1] # the point in the loop before -- used to calculate path equation
for waypoint in pathArray: # gos through and draws lines between waypoints
    if(previousPoint[0] == -1): # set up to not run for orgin
        previousPoint = waypoint
    else:
        if(waypoint[1] != previousPoint[1]): # solves a divide by zero issue
            slope = (waypoint[0] - previousPoint[0]) / (waypoint[1] - previousPoint[1])
            #pathEquation: yValue = slope*(xValue - previousPoint[1]) + previousPoint[0]
            
            pathCountIncrement = 0.2
            if(waypoint[1] > previousPoint[1]): # set up to account for backwards motion 
                xValue = previousPoint[1] + pathCountIncrement / 2

                while(xValue < waypoint[1]): # progresses down the line -- marks boxes black if they intersect with the path
                    yValue = slope * (xValue - previousPoint[1]) + previousPoint[0] # gets a x and y for the path at a certian point

                    outputImageBitmap[math.floor(yValue)][math.floor(xValue)] = [0, 0, 255] # its ok oto mark the same box black twice
                    xValue = xValue + pathCountIncrement
            else:
                pathCountIncrement = -pathCountIncrement
                xValue = previousPoint[1] + pathCountIncrement / 2

                while(xValue > waypoint[1]): # progresses down the line -- marks boxes black if they intersect with the path
                    yValue = slope * (xValue - previousPoint[1]) + previousPoint[0] # gets a x and y for the path at a certian point

                    outputImageBitmap[math.floor(yValue)][math.floor(xValue)] = [0, 0, 255] # its ok to mark the same box black twice
                    xValue = xValue + pathCountIncrement
        else: #line needs to be verical
            if(waypoint[0] > previousPoint[0]): # if the line need to go up
                yCounter = previousPoint[0]
                while(yCounter <= waypoint[0]): # marks the boxes to create the line
                    outputImageBitmap[yCounter][waypoint[1]] = [0, 0, 255]
                    yCounter = yCounter + 1
            else: # line need to go down
                yCounter = previousPoint[0]
                while(yCounter >= waypoint[0]): # marks the boxes to create the line
                    outputImageBitmap[yCounter][waypoint[1]] = [0, 0, 255]
                    yCounter = yCounter - 1

        previousPoint = waypoint



for waypoint in pathArray: # goes through and marks waypoints
    markerZone = [[max(waypoint[0] - k_markerRadius, 0), max(waypoint[1] - k_markerRadius, 0)], [min(waypoint[0] + k_markerRadius, newHeight), min(waypoint[1] + k_markerRadius, newWidth)]] 
    markerZoneYCounter = markerZone[0][0]

    while(markerZoneYCounter < markerZone[1][0]): # clears out the area cover by the waypoint
        markerZoneXCounter = markerZone[0][1]

        while(markerZoneXCounter < markerZone[1][1]): 
            if(np.array_equal(waypoint, orgin)):
                outputImageBitmap[int(markerZoneYCounter)][int(markerZoneXCounter)] = k_orginColor
            else:
                outputImageBitmap[int(markerZoneYCounter)][int(markerZoneXCounter)] = k_markerColor
            markerZoneXCounter = markerZoneXCounter + 1

        markerZoneYCounter = markerZoneYCounter + 1

outputImage = Image.fromarray(outputImageBitmap)
outputImage.save("Output.jpg")

print("Scaling path")

scaledPathArray = [] # the transposed path array but scale to meters
for waypoint in pathArray: # creates a copy of the path array but relative to the orgin
    scaledPathArray.append([(waypoint[0] - orgin[0]) * k_pixelScalingFactor, (waypoint[1] - orgin[1]) * k_pixelScalingFactor])

print("Converting path to code")
codePathArray = []
if(k_convertPathToCode):
    for waypoint in scaledPathArray: 
        codePathArray.append("wayPoints.add(new double[] { " + str(-waypoint[0]) + " , " + str(waypoint[1]) + " });\n")
        #Course manager understands points to be [x,y] but the robot starts facing sideways so [y,x]
        # the course on amperage is alos mirrored so x is -x

print("Generating output text file")

printArray = scaledPathArray
if(k_convertPathToCode):
    printArray = codePathArray

file = open("outputPath.txt", "w") # open file and truncate

for line in printArray:
    file.write(line)
file.close()





    
            

    

