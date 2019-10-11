#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

class Point:
    x = 0
    y = 0
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Drone:
    position = Point(0,0)
    yaw = 0
    def __init__(self, position, yaw):
        self.position = position
        self.yaw = yaw

class Super_point:
    x = 0
    y = 0
    z = 0
    yaw = 0
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

def points_around_windmill(drone, windmill_position):
    numberOfPoints = 3
    degreesBetween = 360/3
    #radius = RADIUS_AROUND_WINDMILL
    radius = 1
    Ax = windmill_position.x
    Ay = windmill_position.y
    Bx = drone.position.x
    By = drone.position.y
    points = []
    for i in range(1, numberOfPoints):
        print("hei")
        alpha = math.radians(degreesBetween*i)
        Cx=Ax+(Bx-Ax)*math.cos(alpha)-(By-Ay)*math.sin(alpha)
        #Cx=Ax+(Bx−Ax)*math.cos(alpha)−(By−Ay)*math.sin(alpha)
        Cy=Ay+(Bx-Ax)*math.sin(alpha)+(By-Ay)*math.cos(alpha)
        #Cy=Ay+(Bx−Ax)*math.sin(alpha)+(By−Ay)*math.cos(alpha)
        points.append(Super_point(Cx, Cy, 0))
    return points


droneposition = Point(0,0)
drone = Drone(droneposition, 0)
windmill_pos = Point(0,1)

point = points_around_windmill(drone, windmill_pos)
for i in range(0, len(point)):
    print("Point")
    print(i)
    print(point[i].x)
    print(point[i].y)
    print(math.degrees(point[i].yaw))
