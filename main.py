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

TASK2 = True
RUST_THRESHOLD = 20


class Starting_mission(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['startup_complete'],
                            input_keys=['path', 'drone'],
                            output_keys=['path', 'drone'])

    def execute(self, userdata):

        userdata.drone.activate()
        userdata.drone.takeoff(TAKEOFF_HEIGHT)

        # Gets windmill position and makes a path that begins and ends at launch site
        windmill_positions = get_windmill_positions()
        home_point = Super_point(0, 0, 0)
        windmill_positions.append(home_point)
        windmill_positions.insert(0, home_point)
        userdata.path = make_path.make_path(windmill_positions)

        return 'startup_complete'


class Flying_to_target(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['arrived_at_landing_pos','arrived_at_windmill'],
                            input_keys=['path', 'drone', 'current_windmill', 'windmill_path'],
                            output_keys=['path', 'drone', 'current_windmill', 'windmill_path'])

    def execute(self, userdata):

        target = userdata.path.pop()

        if target.x == 0 and target.y == 0:     # target is launch pad
            userdata.drone.set_target(target.x, target.y, LANDING_HEIGHT)
        else:
            # Modify target such that drone stops in front of windmill
            userdata.current_windmill = target
            x_new, y_new = get_closer_target(userdata.drone, target, RADIUS_AROUND_WINDMILL)

            # find desired yaw
            target_yaw = yaw_towards_windmill(Super_point(x_new, y_new, 0), target)

            userdata.drone.set_target(x_new, y_new, OPERATING_HEIGHT, yaw = target_yaw)

            # calculate best path around windmill 
            userdata.windmill_path = points_around_windmill(target, userdata.current_windmill)
            if next_target_is_left(userdata.drone.position, target, userdata.path[0]):
                # flipps order of points, so that drone exits close to next target
                tmp = userdata.windmill_path.pop(-1)     
                userdata.windmill_path.insert(1, tmp)




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
                            input_keys=['drone', 'rust_score_dict', 'rust_reports', 'current_windmill', 'windmill_path'],
                            output_keys=['drone', 'rust_score_dict', 'rust_reports', 'current_windmill', 'windmill_path'])

    def execute(self, userdata):
        images = []
        rust_images = []
        has_rust = False
        first_target = True

        for target in userdata.windmill_path:

            if not first_target:

                # Fly to target
                userdata.drone.set_target(target.x, target.y, OPERATING_HEIGHT, yaw=target.yaw)

                while not is_at_target(userdata.drone) :
                    continue

            first_target = False

            # Take and analyse photo
            if TASK2:
                img = userdata.drone.camera.image
                if rust_score(img) > RUST_THRESHOLD:
                    # not necessary to check windmill further
                    has_rust = True
                    rust_report = build_rust_report_message(userdata.current_windmill, has_rust, [img])
                    rospy.loginfo("Found rust on windmill")
                    send_single_inspection_report(rust_report)
                    return 'inspection_complete'

            else:
                images.append(userdata.drone.camera.image)



        if TASK2:
            rust_report = build_rust_report_message(userdata.current_windmill, has_rust, rust_images)
            rospy.loginfo("Found no rust on windmill")
            send_single_inspection_report(rust_report)

        else:
            photo_id = 0
            score_sum = 0
            for img in images:
                score = rust_score(img)

                # Lagre bilde og score
                windmill_position = userdata.current_windmill
                save_photos(windmill_position, photo_id, img, score)
                photo_id = photo_id + 1

                if score > RUST_THRESHOLD:
                    has_rust = True
                    rust_images.append(img)
                    score_sum = score_sum + score

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
            print("sortet_rust_reports")
            #print(sortet_rust_reports)
            print("userdata.rust_score_dict")
            print(userdata.rust_score_dict)
            rospy.loginfo("Sending rust reports for task 3")
            print("Report.score:")
            for report in sortet_rust_reports:
                print("hei")
                print(report.position)
                #print()


        while (abs(userdata.drone.velocity.z) > 1.5):
            print(userdata.drone.velocity)  # TODO: fjern denne
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
    sm.userdata.windmill_path = []


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
