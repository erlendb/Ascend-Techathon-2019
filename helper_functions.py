#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

TAKEOFF_HEIGHT = 2
OPERATING_HEIGHT = 8.5
LANDING_HEIGHT = 16
RADIUS_AROUND_WINDMILL = 18

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


def is_collision(drone, current_windmill):
    detection_radius = RADIUS_AROUND_WINDMILL-10
    distance_to_target = ((current_windmill.x - drone.position.x)**2 +
                          (current_windmill.y - drone.position.y)**2)**0.5

    if distance_to_target < detection_radius:
        return True

    return False


def is_almost_at_target(drone):

    radius = 6  # Radius for almost at target

    distance_to_target = ((drone.target.x - drone.position.x)**2 +
                          (drone.target.y - drone.position.y)**2 +
                          (drone.target.z - drone.position.z)**2)**0.5

    if distance_to_target < radius:
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


def points_around_windmill(position, windmill_pos, total_points=6, cut=2.0/3.0, radius=RADIUS_AROUND_WINDMILL, clockwise=True):

    t = list((range(0, 360, int(360/total_points))))
    point = []
    x = []
    y = []

    normal_radius = radius
    short_radius = radius / 1.5

    for i in range(len(t)):  # Lager punkter med radius radius fra origo
        if i % 2 == 1:
            radius = short_radius
        else:
            radius = normal_radius

        if clockwise:
            x.append(radius*(math.sin(math.radians(t[i]))))
        else:
            x.append(radius*(-math.sin(math.radians(t[i]))))
        y.append(radius*(math.cos(math.radians(t[i]))))

    index_array = list((range(int(total_points/total_points), int(math.ceil(total_points*cut)) + 1, int(total_points/total_points))))  # Første punkt er drone_pos!! (evt endre på første )

    # Første punkt er drone_pos!!
    # Finner vinkel for verdenskoordinater til dronekoordinater
    angle = angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), (position.x, position.y))

    point.append(Super_point(position.x, position.y, math.radians(angle-90)))  # Første punkt er drone_pos!!

    for i in index_array:
        point.append(Super_point((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y, math.radians(angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), ((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y))-90)))

    return point


def score(report, score_dict):
    return score_dict[(report.position.x, report.position.y)]

def get_dist(point1, point2):
    distance = ((point1.x - point2.x)**2 + (point1.y - point2.y)**2)**0.5
    return distance


def direct_travel_allowed(drone_pos, current_windmill, next_windmill):
    angle = angle3pt((drone_pos.x, drone_pos.y), (current_windmill.x, current_windmill.y), (next_windmill.x, next_windmill.y))
    if angle > 120 or (angle > 240 and angle <= 360):
        return True
    else:
        return False
