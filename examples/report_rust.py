#!/usr/bin/env python

from __future__ import print_function

import rospy

from dronelib import SimDrone
from util import build_rust_report_message, send_single_inspection_report, send_total_inspection_report, get_windmill_positions


def is_at_target(drone):
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

    # Initialize drone
    drone = SimDrone()
    drone.activate()
    drone.takeoff()

    # Get data about all the windmills
    windmill_positions = get_windmill_positions()

    # Fly to the left
    drone.set_target(-5, 0, 20)

    # Run until we arrive at the target
    # rospy.Rate makes sure this loop runs at 30 Hz, don't remove this
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

        if is_at_target(drone):
            break


    # Take a picture 
    img = drone.camera.image

    # Reporting rust for task 2
    # 
    # In task 2, you only have to say yes/no there is rust on the windmill 

    # We build a rust report for the given windmill and declare that it has rust and attach an image

    current_windmill = windmill_positions[0]    # which windmill we are currently inspecting
    has_rust = True                             # the current windmill has rust
    images = [img]                              # the images should be of rust spots

    rust_report = build_rust_report_message(current_windmill, has_rust, images)
    rospy.loginfo("Sending rust report for task 2")
    send_single_inspection_report(rust_report)

    # Reporting rust for task 3
    # 
    # In task 3, you have to sort the windmills after rust corrosion severity  

    # rust_report_1 = build_rust_report_message(windmill_positions[0], True, [img])
    # rust_report_2 = build_rust_report_message(windmill_positions[1], True, [img])

    # Sort the reports after rust severity
    # in this case we assume the windmill described by report 1 had more
    # corrosion than the one described by report 2.
    # sorted_reports = [rust_report_1, rust_report_2]

    # rospy.loginfo("Sending rust reports for task 3")
    # send_total_inspection_report(sorted_rust_reports)

    # Go back to origin
    drone.set_target(0, 0, 20)
    while not rospy.is_shutdown():
        rate.sleep()
        if is_at_target(drone):
            break

    drone.land()
    drone.deactivate()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
