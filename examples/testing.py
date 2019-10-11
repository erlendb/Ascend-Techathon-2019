#!/usr/bin/env python

from __future__ import print_function

from math import pi, cos, sin

import rospy

from dronelib import SimDrone


def main():
    # Init ROS node
    rospy.init_node('task', anonymous=True)

    # Initialize drone
    drone = SimDrone()
    drone.activate()
    drone.takeoff()

    # Move in a circle forever
    radius = 10
    angle = 0
    orbital_period = 20
    omega = 2*pi/orbital_period
    hz = 20.0
    t, dt = 0, 1/hz
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        rate.sleep()
        x = radius*cos(omega*t)
        y = radius*sin(omega*t)
        t += dt

        drone.set_target(x, y)

    drone.land()
    drone.deactivate()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
