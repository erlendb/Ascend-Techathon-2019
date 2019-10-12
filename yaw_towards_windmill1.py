
# Returnerer jaw mot windmøllen
def yaw_towards_windmill(drone_pos, windmill_pos):
    angle = angle3pt((windmill_pos.x, windmill_pos.y + 1), (windmill_pos.x, windmill_pos.y), (drone_pos.x, drone_pos.y))-90
    yaw = math.radians(angle)
    return yaw



