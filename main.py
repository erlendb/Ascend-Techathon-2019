 #!/usr/bin/env python

import rospy
import smach
import smach_ros

from dronelib import SimDrone
from tsp_solver.greedy import solve_tsp



class Starting_mission(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['startup_complete'],
                            input_keys=['path', 'drone'],
                            output_keys=['path', 'drone'])

    def execute(self, userdata):

        userdata.drone.activate()
        userdata.drone.takeoff()

        windmill_positions = get_windmill_positions()
        userdata.path = make_path(windmill_positions)

        return 'startup_complete'


class Flying_to_target(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['arrived_at_landing_pos','arrived_at_windmill'],
                            input_keys=['path', 'drone'],
                            output_keys=['path', 'drone'])

    def execute(self, userdata):
        target = userdata.path.pop()
        userdata.drone.set_target(target.x, target.y)

        while not is_at_target(userdata.drone):
            continue # TODO: sleep?


        if not userdata.path: # no more targets in path => drone is back home 
            return 'arrived_at_landing_pos'
        else:
            return 'arrived_at_windmill'


class Inspecting(smach.State): 
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['inspection_complete'],
                            input_keys=['drone', 'rust_score_dict', 'rust_reports'],
                            output_keys=['drone', 'rust_score_dict', 'rust_reports'])

    def execute(self, userdata):
        current_windmill = userdata.drone.target
        sub_path = points_around_windmill(userdata.drone.position, current_windmill)
        images = []
        rust_scores = []


        for target in sub_path:

            # Fly to target
            userdata.drone.set_target(target.x, target.y, yaw=target.yaw) # TODO: targets as points or setpoints?
            while not is_at_target(userdata.drone, 1):
                continue # TODO: sleep?

            # Take and analyse photo
            images.append(userdata.drone.camera_image)
            rust_scores.append(analyse_photo(images[-1]))


        # Evaluate if/how much rust on windmill
        RUST_THRESHOLD = 10
        rust_score = sum(rust_scores)
        has_rust = False
        if rust_score > RUST_THRESHOLD:
            has_rust = True

        # Save rust score for later sorting
        userdata.rust_score_dict[current_windmill] = rust_score

        # Save report for current windmill
        # userdata.rust_reports.append(build_rust_report_message(current_windmill, has_rust, images))

        return 'inspection_complete'


class Ending_mission(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['mission_ended'],
                            input_keys=['path', 'drone'],
                            output_keys=['path'])

    def execute(self, userdata):
        userdata.drone.land()

        # Rapportere
        # rospy.loginfo("Sending rust reports for task 3")
        # send_total_inspection_report(sorted_rust_reports)

        while (userdata.drone.z > 0):
            pass
        userdata.drone.deactivate()

        return 'mission_ended'


def make_path(points):    
    distances = []
    for i in range(0, len(points)):
        sub_distances = []
        for j in range(0, len(points)):
            distance = ((points[j].x - points[i].x)**2 + (points[j].y - points[i].y)**2)**0.5
            sub_distances.append(distance)
        distances.append(sub_distances)

    last_index = len(points) - 1
    path_indexes = solve_tsp(distances, endpoints = (0,last_index) )

    path = []
    for i in range(0, len(points)):
        index = path_indexes[i]
        path.append(points[index])

    return path


def analyse_photo(img):   # TODO: implement photo taking and analysis
    """
    Returns a score for how much rust there is in the image 
    """
    pass


def points_around_windmill(drone_pos, windmill_pos):
    return None


def is_at_target(drone, distance_to_target=10):
    """
    returns True when drone is within a set distance to target,
    and return False otherwise.
    """
    pass



def main():
    rospy.init_node('brute_force_one')

    # TODO: set up smach correctly with all states

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['mission_ended'])

    sm.userdata.drone = SimDrone()
    sm.userdata.path = []
    sm.userdata.rust_reports = []
    sm.userdata.rust_score_dict = {}


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
								transitions={'mission_ended':'mission_ended'}, # ???????????
								remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
