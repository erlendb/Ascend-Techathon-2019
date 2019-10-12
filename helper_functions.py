#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

TAKEOFF_HEIGHT = 1
OPERATING_HEIGHT = 10
LANDING_HEIGHT = 16
RADIUS_AROUND_WINDMILL = 14

class Super_point:
    x = 0
    y = 0
    z = 0
    yaw = 0
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

def is_at_target(drone):
    distance_to_target = ((drone.target.x - drone.position.x)**2 +
                          (drone.target.y - drone.position.y)**2 +
                          (drone.target.z - drone.position.z)**2)**0.5
    velocity = (drone.velocity.x**2 + drone.velocity.y**2 + drone.velocity.z**2)**0.5

    if distance_to_target < 0.5 and velocity < 0.2:
        return True

    return False


# Returnerer jaw mot windmøllen
def yaw_towards_windmill(drone_pos, windmill_pos):
    angle = angle3pt((windmill_pos.x, windmill_pos.y + 1), (windmill_pos.x, windmill_pos.y), (drone_pos.x, drone_pos.y))-90
    yaw = math.radians(angle)
    return yaw


def get_closer_target(drone, target, distance=10):
    diff = (drone.position.x - target.x, drone.position.y - target.y)
    lenght = (diff[0]**2 + diff[1]**2)**0.5
    unit_diff = (diff[0] / lenght, diff[1] / lenght)
    dx = unit_diff[0] * distance
    dy = unit_diff[1] * distance
    return (target.x + dx, target.y + dy)


def angle3pt(a, b, c):
    """Counterclockwise angle in degrees by turning from a to c around b
        Returns a float between 0.0 and 360.0"""
    ang = math.degrees(
        math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
    return ang + 360.0 if ang < 0.0 else ang

## Returnerer en liste med points(x, y, yaw) med lenght lik total_points. Maa kjøres i første bildeposisjon.
def points_around_windmill(drone_pos, windmill_pos): # need: import math
    radius = RADIUS_AROUND_WINDMILL          # Avstand fra fyrtårn [m]
    total_points = 3    # Antall punkter rundt fyrtårnet. Funker med 1, 2, 3, 4, 6, 12.
    t = list((range(0, 360, int(360/12))))
    point = []
    x = []
    y = []
    for i in range(len(t)):  # Lager 12 punkter med radius radius fra origo
        x.append(radius*(math.sin(math.radians(t[i]))))
        y.append(radius*(math.cos(math.radians(t[i]))))
    index_array = list((range(int(12/total_points), 12, int(12/total_points))))  # Første punkt er drone_pos!! (evt endre på første )
     # Første punkt er drone_pos!!
     # Finner vinkel for verdenskoordinater til dronekoordinater
    angle = angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), (drone_pos.x, drone_pos.y))
    point.append(Super_point(drone_pos.x, drone_pos.y, math.radians(angle-90)))  # Første punkt er drone_pos!!
    for i in index_array:
        point.append(Super_point((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y, math.radians(angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), ((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y))-90)))
    return point


def score(report, score_dict):
    return score_dict[(report.position.x, report.position.y)]


def next_target_is_left(drone_pos, target_pos, next_pos):
    """
    returns true if next_pos is left (from drone perspektive) of the line between drone_pos
    and target_pos
    """
    a = drone_pos
    b = target_pos
    c = next_pos
    return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0

