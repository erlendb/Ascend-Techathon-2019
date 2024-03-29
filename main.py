#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import math
from operator import attrgetter

from dronelib import SimDrone
from tsp_solver.greedy import solve_tsp
from util import get_windmill_positions, build_rust_report_message, send_total_inspection_report, send_single_inspection_report
from is_rust import is_rust
from rust_score import rust_score
import make_path
from save_photos import save_photos
from helper_functions import *



class Starting_mission(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['startup_complete'],
                            input_keys=['path', 'drone', 'windmill_list'],
                            output_keys=['path', 'drone', 'windmill_list'])

    def execute(self, userdata):
        userdata.drone.activate()
        userdata.drone.takeoff(TAKEOFF_HEIGHT)

        # Gets windmill position and makes a path that begins and ends at launch site
        windmill_positions = get_windmill_positions()
        userdata.windmill_list = windmill_positions
        home_point = Super_point(0, 0, 0)
        windmill_positions.append(home_point)
        windmill_positions.insert(0, home_point)
        userdata.path = make_path.make_path(windmill_positions)

        return 'startup_complete'


class Flying_to_target(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['arrived_at_landing_pos','arrived_at_windmill'],
                            input_keys=['path', 'drone', 'current_windmill', 'sub_path', 'next_windmill'],
                            output_keys=['path', 'drone', 'current_windmill', 'sub_path', 'next_windmill'])

    def execute(self, userdata):

        target = userdata.path.pop()

        if target.x == 0 and target.y == 0: # target is launch pad
            userdata.drone.set_target(target.x, target.y, LANDING_HEIGHT)
        else:
            # Save windmill pos for use in Inspection
            userdata.current_windmill = target
            userdata.next_windmill = userdata.path[-1]

            # Modify target such that drone stops in front of windmill
            x_new, y_new = get_closer_target(userdata.drone, target, RADIUS_AROUND_WINDMILL)

            # find desired yaw
            target_yaw = yaw_towards_windmill(Super_point(x_new, y_new, 0), target)

            userdata.drone.set_target(x_new, y_new, OPERATING_HEIGHT, yaw = target_yaw)

            next_target = userdata.path[-1]
            angle = angle3pt((target.x, target.y), (userdata.drone.position.x, userdata.drone.position.y), (next_target.x, next_target.y))

            if math.sin(math.radians(angle)) < 0:
                userdata.sub_path = points_around_windmill(Super_point(x_new, y_new, 0), userdata.current_windmill, clockwise=True)
            else:
                userdata.sub_path = points_around_windmill(Super_point(x_new, y_new, 0), userdata.current_windmill, clockwise=False)


        # TODO: collision avoidance

        while not is_at_target(userdata.drone):
            continue

        if not userdata.path:   # no more targets in path => drone is back home
            return 'arrived_at_landing_pos'
        else:
            return 'arrived_at_windmill'


class Inspecting(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['inspection_complete'],
                            input_keys=['drone', 'rust_score_dict', 'rust_reports', 'current_windmill', 'sub_path', 'next_windmill'],
                            output_keys=['drone', 'rust_score_dict', 'rust_reports', 'current_windmill', 'sub_path', 'next_windmill'])

    def execute(self, userdata):
        images = []
        has_rust = False
        rust_images = []
        first_target = True

        # Defines points where phots should be taken
        picture_target = [userdata.sub_path[0]]
        picture_target.append(userdata.sub_path[int(len(userdata.sub_path)/2)])
        picture_target.append(userdata.sub_path[-1])


        for target in userdata.sub_path:

            if not first_target:
                take_photo = False
                for pt in picture_target:
                    if target.x == pt.x and target.y == pt.y:
                        take_photo = True

                if take_photo:
                    userdata.drone.set_target(target.x, target.y, OPERATING_HEIGHT, yaw=target.yaw)
                    while not is_at_target(userdata.drone):
                        continue
                else:
                    userdata.drone.set_target(target.x, target.y, OPERATING_HEIGHT, yaw=target.yaw)
                    while not is_almost_at_target(userdata.drone):
                        continue
                    continue

            first_target = False

            # Take and analyse photo
            if TASK2:
                img = userdata.drone.camera.image
                if rust_score(img, weighing=False) > RUST_THRESHOLD:
                    # not necessary to check windmill further
                    has_rust = True
                    rust_report = build_rust_report_message(userdata.current_windmill, has_rust, rust_images)
                    rospy.loginfo("Found rust on windmill - task2")
                    send_single_inspection_report(rust_report)
                    picture_target = []

                    # Kanskje fly direkte videre
                    if direct_travel_allowed(userdata.drone.position, userdata.current_windmill, userdata.next_windmill):
                        if first_target:
                            x_new, y_new = get_closer_target(userdata.drone, target, 12)
                            userdata.drone.set_target(x_new, y_new)
                            while not is_at_target(userdata.drone):
                                continue
                        return 'inspection_complete'

            else:
                images.append(userdata.drone.camera.image)



        if TASK2:
            rust_report = build_rust_report_message(userdata.current_windmill, has_rust, rust_images)
            rospy.loginfo("Found no rust on windmill - task 2")
            send_single_inspection_report(rust_report)
            return 'inspection_complete'

        else:

            # Start flying to next windmill, because the image analysis takes a while
            userdata.drone.set_target(userdata.next_windmill.x, userdata.next_windmill.y, OPERATING_HEIGHT)

            score_sum = 0
            photo_id = 0
            windmill_position = userdata.current_windmill

            # Sjekk bilder fra windillen etter rust
            scorearray = []
            for img in images:
                score = rust_score(img)
                scorearray.append(score)

                if score > RUST_THRESHOLD:
                    has_rust = True
                    rust_images.append(img)
                    score_sum = score_sum + score

            photo_id = 0
            for img in images:
                # Lagre bilde og score
                #save_photos(windmill_position, photo_id, img, scorearray[photo_id], score_sum)
                photo_id = photo_id + 1


            # Save rust score for later sorting
            userdata.rust_score_dict[(windmill_position.x, windmill_position.y)] = score_sum

            # Save report for current windmill
            userdata.rust_reports.append(build_rust_report_message(windmill_position, has_rust, rust_images))

        return 'inspection_complete'


class Ending_mission(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['mission_ended'],
                            input_keys=['path', 'drone', 'rust_reports', 'rust_score_dict'],
                            output_keys=['path', 'drone', 'rust_reports', 'rust_score_dict'])

    def execute(self, userdata):
        userdata.drone.land()

        if not TASK2:
            # sorts reports based on return value from function score
            reports = userdata.rust_reports
            sortet_rust_reports = sorted(reports, reverse=True, key=lambda report: score(report, userdata.rust_score_dict))
            send_total_inspection_report(sortet_rust_reports)
            rospy.loginfo("Sending rust reports for task 3")


        while (abs(userdata.drone.velocity.z) > 1.5):
            continue
        userdata.drone.deactivate()

        return 'mission_ended'






def main():
    rospy.init_node('brute_force_one')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['mission_ended'])

    sm.userdata.drone = SimDrone()
    sm.userdata.path = []
    sm.userdata.rust_reports = []
    sm.userdata.rust_score_dict = {}
    sm.userdata.current_windmill = None
    sm.userdata.next_windmill = None
    sm.userdata.sub_path = []
    sm.userdata.windmill_list = []


    # Open the container
    with sm:
        # Add states to the container

		# TODO:	fiks/fjern remapping
		#		transition ut fra ending_mission
		#		outomes fra inspecting
        smach.StateMachine.add('starting_mission', Starting_mission(),
                                transitions={'startup_complete':'flying_to_target'},
                                remapping={'foo_counter_in':'sm_counter', 'foo_counter_out':'sm_counter'})

        smach.StateMachine.add('flying_to_target', Flying_to_target(),
                                transitions={'arrived_at_landing_pos':'ending_mission', 'arrived_at_windmill':'inspecting'},
                                remapping={'bar_counter_in':'sm_counter'})

    	smach.StateMachine.add('inspecting', Inspecting(),
								transitions={'inspection_complete':'flying_to_target'},
								remapping={'foo_counter_in':'sm_counter', 'foo_counter_out':'sm_counter'})

    	smach.StateMachine.add('ending_mission', Ending_mission(),
								transitions={'mission_ended':'mission_ended'}, # transition -> transition ????
								remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    main()
