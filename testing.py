#!/usr/bin/env python

import rospy
import smach
import smach_ros


from dronelib import SimDrone
from tsp_solver.greedy import solve_tsp
from util import get_windmill_positions


RELATIVE_FLYING_HEIGT = 5

class Super_point:
    x = 0
    y = 0
    z = 0
    yaw = 0
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


class Starting_mission(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['startup_complete'],
                            input_keys=['path', 'drone'],
                            output_keys=['path', 'drone'])

    def execute(self, userdata):

        userdata.drone.activate()
        userdata.drone.takeoff(RELATIVE_FLYING_HEIGT)

        rospy.loginfo(userdata.drone.yaw)

        # Gets windmill position and makes a path that begins and ends at launch site
        windmill_positions = get_windmill_positions()
        windmill_positions.pop()
        windmill_positions.pop()
        windmill_positions.pop()
        windmill_positions.pop()
        windmill_positions.pop()
        windmill_positions.pop()
        windmill_positions.pop()
        windmill_positions.pop()
        home_point = Super_point(0, 0, 0)
        windmill_positions.append(home_point)
        windmill_positions.insert(0, home_point)
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

        if target.x == 0 and target.y == 0:
            pass

        userdata.drone.set_target(target.x, target.y)

        # TODO: dehackify
        while distance_to_target(userdata.drone) > 20:
            continue # TODO: sleep?
        current_pos = userdata.drone.position
        userdata.drone.set_target(current_pos.x, current_pos.y)

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
            while not is_at_target(userdata.drone):
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
                            output_keys=['path', 'drone'])

    def execute(self, userdata):
        userdata.drone.land()

        # Rapportere
        # rospy.loginfo("Sending rust reports for task 3")
        # send_total_inspection_report(sorted_rust_reports)

        while (abs(userdata.drone.velocity.z) > 0):
            continue
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
    for i in range(1, len(points)):
        index = path_indexes[i-1]
        path.append(points[index])

    return path


def analyse_photo(img):   # TODO: implement photo taking and analysis
    """
    Returns a score for how much rust there is in the image
    """
    pass


def points_around_windmill(drone_pos, windmill_pos):
    return []


def distance_to_target(drone):
    """
    returns distance to target
    """
    return  ((drone.target.x - drone.position.x)**2 +
                          (drone.target.y - drone.position.y)**2 +
                          (drone.target.z - drone.position.z)**2)**0.5


def isAtTarget(drone):
    distance_to_target = ((drone.target.x - drone.position.x)**2 +
                          (drone.target.y - drone.position.y)**2 +
                          (drone.target.z - drone.position.z)**2)**0.5
    velocity = (drone.velocity.x**2 + drone.velocity.y**2 + drone.velocity.z**2)**0.5

    if distance_to_target < 0.5 and velocity < 0.2:
        return True

    return False



def main():
    rospy.init_node('brute_force_one')

    # TODO: set up smach correctly with all states


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['mission_ended'])

    sm.userdata.drone = SimDrone()
    sm.userdata.path = []
    sm.userdata.rust_reports = []
    sm.userdata.rust_score_dict = {}

    img = drone.camera_image
    roslog(img)

    '''
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
    '''


if __name__ == '__main__':
    main()
