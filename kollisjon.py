 while not is_at_target(userdata.drone):
            for current_windmill in windmill_list
                if is_collision(drone_position, current_windmill)  # Sett inn og dobbelsjekk funskjonen
                    sub_path = points_around_windmill(userdata.drone.position, current_windmill_position, cut=1.0/2.0, total_points=4) # Har lyst å sette inn points og cut som input, vi må tenke på jaw
                    for new_target in sub_path:
                        userdata.drone.set_target(new_target.x, new_target.y) # VIl holde samme yaw?
                        while not is_almost_at_target(userdata.drone, 1):  # Sett inn og dobbelsjekk funksjonen
                            continue  # er continue på riktig plass??
        continue