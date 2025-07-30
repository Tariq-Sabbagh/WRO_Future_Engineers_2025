 
VIDEO_SOURCE = "D:\\WRO\\video\\my_video_title.mp4"
VIDEO_SOURCE_1 = "D:\\WRO\\video\\my_video_title1.mp4"
VIDEO_SOURCE_2= "D:\\WRO\\video\\my_video_title2.mp4"
VIDEO_SOURCE_3= "D:\\WRO\\video\\my_video_title3.mp4"

lotType = "light" #0 for dark purple, 1 for magenta
rMagenta = [[0, 0, 0], [0, 0, 0]]
rRed = [[0, 130, 0], [123, 255, 90]]
rGreen = [[0, 0, 150], [120, 115, 255]]
rOrange = [[120, 115, 0], [255, 255, 130]]
rBlue = [[90, 129, 135], [255, 255, 255]]
rBlack = [[0, 73, 0], [90, 142, 148]]


redTarget = 10 
greenTarget = 590


#ROI1: for finding left lane
#ROI2: for finding right lane
#ROI3: for finding signal pillars
#ROI4: for detecting blue and orange lines on mat
#ROI5: helps with turns at corners
# order: x1, y1, x2, y2
ROI1 = [0, 175, 330, 265] # 165, 255
ROI2 = [330, 175, 640, 265]
ROI3 = [redTarget+50 , 120, greenTarget-50 , 480] #+- 40 [redTarget - 40, 110, greenTarget + 40, 335]
ROI4 = [200, 420, 400, 480] #250, 300
ROI5 = [0, 0, 0, 0] #270, 120, 370, 140


#booleans for tracking whether car is in a left or right turn
lTurn = False
rTurn = False
 
straightConst = 90 
tempParking = False
endConst = 30

kp = 0.005 #proportional value for PD steering 0.015
kd = 0.01 #derivative value for PD steering 0.01
cKp = 0.15 #value of proportional for proportional steering for avoiding signal pillars
cKd = 0.15 #value of derivative for proportional and derivative sterring for avoiding signal pillars
cy = 0.15 #value used to affect pd steering based on how close the pillar is based on its y coordinate

exitThresh = 6000 #if area of both lanes is over this threshold car exits a turn

angle = 90 #variable for the current angle of the car
prevAngle = angle #variable tracking the angle of the previous iteration
tDeviation = 40 #value used to calculate the how far left and right the car turns during a turn, could actually be 50
sharpRight = straightConst - tDeviation #the default angle sent to the car during a right turn
sharpLeft = straightConst + tDeviation #the default angle sent to the car during a left turn

speed = 200
# speed2 = 200 #variable for initial speed of the car
reverseSpeed = 100 #variable for speed of the car going backwards

aDiff = 0 #value storing the difference of area between contours
prevDiff = 0 #value storing the previous difference of contours for derivative steering
error = 0 #value containing the difference between a pillar's x coordinate and its target x coordinate
prevError = 0 #stores previous error


t = 0 #tracks number of turns
t2 = 0 #tracks number of turns but updates only when seeing a coloured line at a corner

tSignal = False #boolean detecting whether a coloured line is currently seen, makes sure that a pillar doesn't affect a turn too early


rightArea, leftArea, areaFront, areaFrontMagenta, tArea = 0, 0, 0, 0, 0
maxDist = 320 #no pillar can be detected if further than this value
pillarAtStart = -1
minArea = 10
maxArea =1000000


    #stores whether a turn was counted by seeing a wall or a pillar
eTurnMethod = ""

#boolean used to indicate whether the car has stopped after 3 laps
lapsComplete = False
#set the target x coordinates for each red and green pillar

#variable that keeps track of the target of the last pillar the car has passed
lastTarget = 0
#boolean that is used for when the car has to turn around to the opposite direction so the car can complete the last lap the other way around
reverse = False
#boolean storing the only direction the car is turning during the run
turnDir = "none" 
leftAreaMin =6000
rightAreaMin =6000

#variables to indicate when the car should park and whether it parks on the right or left side
parkingR = False
parkingL = False

#stores a stop time 
sTime = 0

#determines how many number of seconds after sTime are needed to stop
 
#determines the offset from the bottom of the ROI when the car should stop seeing a pillar

#regions of interest


#booleans for tracking whether car is in a left or right turn
lTurn = False
rTurn = False

#  variables for printing 
variables = {
                # "left wall area": leftArea,
                # "right wall area": rightArea,
                #"left wall y": lWallY,
                #"right wall y": rWallY,
                #"left parking lot area": maxAreaL if tempParking else 0,
                # "right parking lot area": maxAreaR if tempParking else 0,
                # "front parking area": areaFrontMagenta,
                #"front wall area": areaFront,
                #"three-point turn status": reverse,
                #"last pillar": pColour,
                #"current pillar": cpColour,
                # "pillar area": cPillar.area, 
                # "turn direction": turnDir[0],
                # "pillar distance": cPillar.dist, 
                # "# turns": t,
                # "t2": t2,
                # "green pillars": num_pillars_g,
                # "red pillars:": num_pillars_r,
                # "turn status": turn_status,
                #"end turn method": eTurnMethod if eTurnMethod == "pillar" or eTurnMethod == "wall" else " ",
                # "tArea": tArea,
                # "angle:": angle,
                #"pillar y": cPillar.y,
                # "pillar at start": pillarAtStart,
                #"frames": frames
            }
             