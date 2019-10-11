#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import math


from dronelib import SimDrone
from tsp_solver.greedy import solve_tsp
from util import get_windmill_positions, build_rust_report_message
from is_rust import is_rust
from rust_score import rust_score


RELATIVE_FLYING_HEIGT = 1
RADIUS_AROUND_WINDMILL = 20

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

        # Gets windmill position and makes a path that begins and ends at launch site
        windmill_positions = get_windmill_positions()   
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
        
        if target.x == 0 and target.y == 0: # target is launch pad
            userdata.drone.set_target(target.x, target.y, 1)
        else:
            # Scare target such that drone stops in front of windmill
            x_new, y_new = get_closer_target(userdata.drone, target, RADIUS_AROUND_WINDMILL)
            userdata.drone.set_target(x_new, y_new)

        # TODO: collision avoidance

        while not is_at_target(userdata.drone):
            continue

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
        sub_path = points_around_windmill(userdata.drone, current_windmill)
        images = []


        for target in sub_path:

            # Fly to target
            userdata.drone.set_target(target.x, target.y, yaw=target.yaw) # TODO: targets as points or setpoints?
            while not is_at_target(userdata.drone):
                continue 

            # Take and analyse photo
            images.append(userdata.drone.camera.image)


        # Evaluate if/how much rust on windmill
        RUST_THRESHOLD = 10

        has_rust = False
        rust_images = []
        for img in images:
            score = rust_score(img)
            if score > RUST_THRESHOLD:
                has_rust = True
                rust_images.append(img)

        # Save rust score for later sorting
        #userdata.rust_score_dict[current_windmill] = rust_score

        # Save report for current windmill
        userdata.rust_reports.append(build_rust_report_message(current_windmill, has_rust, rust_images))

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

        while (abs(userdata.drone.velocity.z) > 0.5):
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

def is_at_target(drone):
    distance_to_target = ((drone.target.x - drone.position.x)**2 +
                          (drone.target.y - drone.position.y)**2 +
                          (drone.target.z - drone.position.z)**2)**0.5
    velocity = (drone.velocity.x**2 + drone.velocity.y**2 + drone.velocity.z**2)**0.5

    if distance_to_target < 0.5 and velocity < 0.2:
        return True

    return False


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
def points_around_windmill(drone, windmill_pos): # need: import math
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
    angle = angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), (drone.position.x, drone.position.y))
    
    point.append(Super_point(drone.position.x, drone.position.y, drone.yaw))  # Første punkt er drone_pos!!
    for i in index_array: 
        point.append(Super_point((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y, math.radians(angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), ((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y))-90)))
    return point




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
