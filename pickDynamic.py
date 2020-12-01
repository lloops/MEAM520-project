import math
import numpy as np
from time import sleep

def computeRad(x, y, color):
    if x==0 and y==0:
        if color == 'red':
            rad = -2.377
        if color == 'blue':
            rad = 0.765
    elif x>0 and y==0:
        rad = 0
    elif x>0 and y>0:
        rad = math.atan(y/x)
    elif x==0 and y>0:
        rad = math.pi/2
    elif x<0 and y>0:
        rad = math.pi + math.atan(y/x)
    elif x<0 and y==0:
        rad = -math.pi
    elif x<0 and y<0:
        rad = -math.pi + math.atan(y/x)
    elif x==0 and y<0:
        rad = -math.pi/2
    elif x>0 and y<0:
        rad = math.atan(y/x)
    return rad

def constructPath(distance):
    q1 = [0.8, 0.08, 0.9, -1.0, -1.57, 30]
    q2 = [0.8, 0.16, 0.8, -1.0, -1.57, 30]
    q3 = [0.8, 0.24, 0.7, -1.0, -1.57, 30]
    q4 = [0.8, 0.32, 0.6, -1.0, -1.57, 30]
    q5 = [0.8, 0.40, 0.4, -0.8, -1.57, 30]
    q6 = [0.8, 0.48, 0.3, -0.6, -1.57, 30]
    q7 = [0.8, 0.56, 0.2, -0.4, -1.57, 30]
    q8 = [0.8, 0.64, 0.15, -0.4, -1.57, 30]
    q9 = [0.8, 0.70, 0.1, -0.4, -1.57, 30]

    path = [q1, q2]

    if distance > 10:
        path.append(q3)
    if distance > 20:
        path.append(q4)
    if distance > 30:
        path.append(q5)
    if distance > 40:
        path.append(q6)
    if distance > 50:
        path.append(q7)
    if distance > 60:
        path.append(q8)
    if distance > 70:
        path.append(q9)

    return path

def extractDynamic(name, pose):
    dpose = []
    for i in range(13):
        if name[i][5]=="d":
            dpose.append(pose[i])
    return dpose

def reachTarget(current, goal):
    return np.linalg.norm(np.array(current) - np.array(goal))<0.03

def computeTime(rad, color):
    if color == 'red':
        if rad>=-2.377:
            t = (3.905 - rad)/0.5
        else:
            t = (-2.377 - rad)/0.5
    if color == 'blue':
        if rad>0.764:
            t = (math.pi*2+0.764 - rad)/0.5
        else:
            t = (0.764 - rad)/0.5
    return t

def move(lynx, path, stepTime):
    for q in path:
        lynx.set_pos(q)
        sleep(stepTime)

    close = [q[0], q[1], q[2], q[3], q[4], 0]
    lynx.set_pos(close)
    sleep(0.5)

def pickDynamic(lynx, color):
    q0 = [0.8, 0, 1.0, -1.0, -1.57, 30]
    lynx.set_pos(q0)

    pos = lynx.get_state()[0]

    print("moving to position...")

    while not reachTarget(pos, q0):
        sleep(0.05)
        pos = lynx.get_state()[0]

    print("ready to grab, waiting for next target...")

    while True:
        sleep(0.05)

        [name, pose, twist] = lynx.get_object_state()
        dpose = extractDynamic(name, pose)

        time = []
        dist = []

        pickCenter = False
        for mat in dpose:
            x = round(mat[0][3], 2)
            y = round(mat[1][3], 2)
            z = round(mat[2][3], 2)

            if z==60 and abs(x)<8 and abs(y)<8:
                pickCenter = True

            if z==60:
                distFromCenter = np.sqrt(x*x + y*y)
                dist.append(distFromCenter)

                rad = computeRad(x, y, color)
                t = computeTime(rad, color)
                time.append(t)

        if len(time)==0:
            print("no more dynamic objects, exit")
            return False

        nextArriveTime = 20
        realTimeFactor = 0.8

        for index in range(len(time)):
            if time[index] < nextArriveTime:
                nextArriveTime = time[index]
                distFromCenter = dist[index]
                distance = 100 - distFromCenter
                threshold = distance/10 * (0.05 / realTimeFactor)

        print("next arrive time:", nextArriveTime)
        print("threshold", threshold)
        print("pickCenter", pickCenter)

        if any(t < threshold and t > 0.5 * threshold for t in time):
            path = constructPath(distance)
            print("path")
            print(path)

            print("grabbing...")
            move(lynx, path, 0.1/realTimeFactor)
            return True

        elif nextArriveTime>3 and pickCenter and max(time)<11:
            path = constructPath(100)
            print("path")
            print(path)

            print("grabbing...")
            move(lynx, path, 0.1/realTimeFactor)
            return True
        
        elif nextArriveTime>5:
            print("waiting time too long, exit...")
            return False
