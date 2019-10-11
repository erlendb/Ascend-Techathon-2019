import math

def angle3pt(a, b, c):
    """Counterclockwise angle in degrees by turning from a to c around b
        Returns a float between 0.0 and 360.0"""
    ang = math.degrees(
        math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
    return ang + 360 if ang < 0 else ang

## Returnerer en liste med points(x, y, yaw) med lenght lik total_points. Må kjøres i første bildeposisjon.
def points_around_windmill(drone_pos, windmill_pos): # need: import math
    radius = 1          # Avstand fra fyrtårn [m]
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
    angle = angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), (drone_pos.x, drone_pos.y))
    point.append(drone_pos)  # Første punkt er drone_pos!!
    for i in index_array: 
        point.append(Point((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y, math.radians(angle3pt((windmill_pos.x, windmill_pos.y + radius), (windmill_pos.x, windmill_pos.y), ((x[i]*math.cos(math.radians(angle)) - y[i]*math.sin(math.radians(angle))) + windmill_pos.x, (y[i]*math.cos(math.radians(angle)) + x[i]*math.sin(math.radians(angle))) + windmill_pos.y))-90)))   # z?????????????????????????????????
    return point