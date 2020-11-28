import math
import numpy as np
from time import sleep

def computeRad(x, y, color):
    if x==0 and y==0:
        if color == 'red':
            rad = -2.379
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

def constructPath():
    q1 = [0.8, 0.08, 0.9, -1.0, -1.57, 30]
    q2 = [0.8, 0.16, 0.8, -1.0, -1.57, 30]
    q3 = [0.8, 0.24, 0.7, -1.0, -1.57, 30]
    q4 = [0.8, 0.32, 0.6, -1.0, -1.57, 30]
    q5 = [0.8, 0.40, 0.4, -0.8, -1.57, 30]
    q6 = [0.8, 0.48, 0.3, -0.6, -1.57, 30]
    q7 = [0.8, 0.56, 0.2, -0.4, -1.57, 30]
    q8 = [0.8, 0.56, 0.2, -0.4, -1.57, 30]

    path = [q1, q2, q3, q4, q5, q6, q7, q8]
    return path

def extractDynamic(name, pose):
    dpose = []
    for i in range(13):
        if name[i][5]=="d":
            dpose.append(pose[i])
    return dpose

def reachTarget(current, goal):
    diff = (current[0]-goal[0])**2 + (current[1]-goal[1])**2 + (current[2]-goal[2])**2 + (current[3]-goal[3])**2 + (current[4]-goal[4])**2 + (current[5]-goal[5])**2
    return np.sqrt(diff)<0.05

def computeTime(rad, color):
    if color == 'red':
        if rad>-2.378:
            t = (3.905 - rad)/0.5
        else:
            t = (-2.378 - rad)/0.5
    if color == 'blue':
        if rad>0.764:
            t = (math.pi*2+0.764 - rad)/0.5
        else:
            t = (0.764 - rad)/0.5
    return t

def pickDynamic(lynx, color):
    q0 = [0.8, 0, 1.0, -1.0, -1.57, 30]
    lynx.set_pos(q0)

    reached_target = False
    while not reached_target:
        sleep(0.05)
        state = lynx.get_state()[0]
        reached_target = reachTarget(state, q0)

    print("ready to grab, waiting for next target...")

    [name, pose, twist] = lynx.get_object_state()
    dpose = extractDynamic(name, pose)

    time = []
    dist = []

    for mat in dpose:
        x = round(mat[0][3], 2)
        y = round(mat[1][3], 2)
        z = round(mat[2][3], 2)

        if z==60:
            distFromCenter = np.sqrt(x*x + y*y)
            dist.append(distFromCenter)

            rad = computeRad(x, y, color)
            t = computeTime(rad, color)
            time.append(t)

    if len(time)==0:
        print("no more dynamic objects, exit")
        return False

    threshold = 0.5
    nextArriveTime = 20

    for index in range(len(time)):
        if time[index] < nextArriveTime:
            nextArriveTime = time[index]
            distFromCenter = dist[index]
            threshold = (100 - distFromCenter)/10 * 0.1

    print("next arrive time:", nextArriveTime)
    print("threshold", threshold)

    if any(t<threshold for t in time):
        print("grabbing...")
        path = constructPath()
        for q in path:
            lynx.set_pos(q)
            sleep(0.3)

        q = [0.8, 0.56, 0.2, -0.4, -1.57, 0]
        lynx.set_pos(q)
        sleep(1)

        return True

    return False
