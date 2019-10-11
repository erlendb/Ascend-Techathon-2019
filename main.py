#!/usr/bin/env python

from __future__ import print_function

import rospy

import random
import math

from dronelib import SimDrone
from util import get_windmill_positions

from ascend_msgs.srv import GlobalMap
from geometry_msgs.msg import Pose, PoseArray

def isAtTarget(drone):
    distance_to_target = ((drone.target.x - drone.position.x)**2 +
                          (drone.target.y - drone.position.y)**2 +
                          (drone.target.z - drone.position.z)**2)**0.5
    velocity = (drone.velocity.x**2 + drone.velocity.y**2 + drone.velocity.z**2)**0.5

    if distance_to_target < 0.5 and velocity < 0.2:
        return True

    return False


def main():
    # Init ROS node
    rospy.init_node('task', anonymous=True)

    # Create map service client
    windmills = get_windmill_positions()

    # Print resources
    for windmill in windmills:
        print("windmill:", windmill.x, windmill.y)

    # Initialize drone
    drone = SimDrone()
    drone.activate()
    drone.takeoff()

    # -- For example code --
    counter = 0
    should_land = False
    drone.set_target(-10, 10)

    # rospy.Rate makes sure this loop runs at 30 Hz, don't remove this
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
        # -------------------------------
        # ------Replace this example-----
        # ---------with your code!-------
        # -------------------------------

        # Do special action if we are at target
        if isAtTarget(drone):
            # Go back to origin
            drone.set_target(0, 0)

            if drone.target.x == 0 and drone.target.y == 0 and isAtTarget(drone):
                break


    drone.land()
    drone.deactivate()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
