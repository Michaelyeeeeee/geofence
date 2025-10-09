import time
import busio
import board
import math

'''
Set of (x,y) coordinates
'''
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

'''
Gets the latitude in coordinate points
    inputs
    str_array - degrees and minutes of latitude from GPS
    index - of latitude in str_array
    
    returns latitude measured by gps
    
'''
def get_latitude(str_array, index):
    latDeg = float(str_array[index][0: 2])
    latMin = float(str_array[index][2: 10]) / 60.0
    latitude = latDeg + latMin
    if str_array[index +1] == "S":
        latitude = -latitude
    return latitude

'''
Gets the longitude in coordinate points
    inputs
    str_array - degrees and minutes of latitude from GPS
    index - of longitude in str_array
    
    returns longitude measured by gps
'''
def get_longitude(str_array, index2):
    longDeg = float(str_array[index2][0: 3])
    longMin = float(str_array[index2][3: 11]) / 60
    longitude = (float(longDeg) + float(longMin))
    if str_array[index2 +1] == "W":
        longitude = -longitude
    return longitude

# To determine if the coordinate/function q lies on the segment pr
def onSegment(p:tuple, q:tuple, r:tuple) -> bool:
    
    if((q[0] <= max(p[0], r[0])) &
       (q[0] >= min(p[0], r[0])) &
       (q[1] <= max(p[1], r[1])) &
       (q[1] >= min(p[1], r[1]))):
        return True
    return False

#Finding orientation
def orientation(p:tuple, q:tuple, r:tuple) -> int:
    
    val = (((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]))) #calculating slope
    
    if (val > 0):
        return 1 #positive slope is clockwise orientation
    elif (val < 0):
        return 2 #negative slope is counterclockwise orientation
    else:
        return 0 #collinear orientation
    
#Determine if line segment p1q1 and p2q2 intersects
def doIntersect(p1, q1, p2, q2):
    
    #looking for orientation
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    
    #General Case: If the orientations are different, they intersect
    if (o1 != o2) and (o3 != o4):
        return True
    
    #Special Case (collinear): If x and y projection intersects, they intersect
    if (o1 == 0) and (onSegment(p1, p2, q1)): #p1, q1 and p2 are collinear and p2 lies on segment p1q1
        return True
    
    if (o2 == 0) and (onSegment(p1, q2, q1)): #p1, q1 and q2 are collinear and q2 lies on segment p1q1
        return True
    
    if (o3 == 0) and (onSegment(p2, p1, q2)): #p2, q2 and p1 are collinear and p1 lies on segment p2q2
        return True
    
    if (o4 == 0) and (onSegment(p2, q1, q2)): #p2, q2 and q1 are collinear and q1 lies on segment p2q2
        return True
    
    return False

#Determine if the point p lies within the polygon
def is_within_polygon(points:list, p:list) -> bool:
    n = len(points)
    if n < 3: #there must be at least 3 points/vertices in a polygon
        return False
    extreme = (float('inf'), p[1]) # casts a ray from point to +inf in the x direction
    decrease = 0 #To calculate number of points where y-coordinate of the polygon is equal to y-coordinate of the point
    count = i = 0 # number of borders the horizontal ray crosses
    
    while True:
        next = (i + 1) % n # next point
        
        # increments number of double counted points
        if(points[i][1] == p[1]):
            decrease += 1
        
        # checks if point is on border
        if (doIntersect(points[i], points[next], p, extreme)):
            if orientation(points[i], p, points[next]) == 0:
                return onSegment(points[i], p, points[next]) 
            count += 1      

        i = next
        if (i == 0):
            break
        # subtract double counted case
        count -= decrease
        
    return (count % 2 == 1)

# Initializes GPS
def initialize_gps():
    uart = busio.UART(baudrate=9600, tx=board.GP12, rx=board.GP13)
    print(uart)
    return uart

# Turns on the LCD
def initialize_lcd(backlight_red, backlight_green, backlight_blue):
    #lcd_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))         # This line specifically should be changed to CircuitPython
    lcd_uart = busio.UART(board.GP4, board.GP5, baudrate=9600)
    lcd_uart.write(b'|')  # write 5 bytes
    lcd_uart.write(b'\x18')  # write 5 bytes
    lcd_uart.write(b'\x08')  # contrast
    lcd_uart.write(b'|')  # Put LCD into setting mode
    lcd_uart.write(b'\x2B')  # Set green backlight amount to 0%
    lcd_uart.write(backlight_red.to_bytes(1, 'big'))  # Set green backlight amount to 0%
    lcd_uart.write(backlight_green.to_bytes(1, 'big'))  # Set green backlight amount to 0%
    lcd_uart.write(backlight_blue.to_bytes(1, 'big'))  # Set blue backlight amount to 0%
    lcd_uart.write(b'|')  # Setting character
    lcd_uart.write(b'-')  # Clear display
    return lcd_uart

# Gets Current Location
def get_gps_location(gps_uart):
    latitude_LL = 0
    longitude_LL = 0
    latitude_GA = 0 
    longitude_GA = 0
    latDivisor = 1
    lonDivisor = 1
    
    while ((latitude_LL==0 and longitude_LL==0) and (latitude_GA==0 and longitude_GA==0)):
            
        time.sleep(0.03)
        str_array = gps_uart.readline()
        
        if str_array is None:
            continue
        try:
            str_array = str_array.decode("utf-8")       # Decodes GPS input
            time.sleep(0.03)
            str_array = str_array.split(",")
            
            if str_array[0] == '$GPGLL':
                latitude_LL = get_latitude(str_array, 1)
                longitude_LL = get_longitude(str_array, 3)

            elif str_array[0] == '$GPGGA':
                latitude_GA = get_latitude(str_array, 2)
                longitude_GA = get_longitude(str_array, 4)
        except (ValueError, IndexError):
            print("valueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire")
    
    if (latitude_LL != 0 and latitude_GA != 0):
        latDivisor = 2
    
    if (longitude_LL != 0 and longitude_GA != 0):
        lonDivisor = 2
    
    latitude_avg = (float(latitude_LL) + float(latitude_GA)) / latDivisor
    longitude_avg = (float(longitude_LL) + float(longitude_GA)) / lonDivisor

    #print("LatIN: " + str(latitude_avg) + " LongIN: " + str(longitude_avg))
    return latitude_avg, longitude_avg


def imu_update(latAvg, longAvg, time_interval, velocity_x, velocity_y, sensor):
    earth_radius = 6378137.0  # Earth's equitorial radius in meters

    imu_acceleration_x, imu_acceleration_y, imu_acceleration_z = sensor.linear_acceleration
    # Velocity Estimation
    velocity_x += imu_acceleration_x * time_interval
    velocity_y += imu_acceleration_y * time_interval

    # Position Estimation
    latitude_change = ((velocity_x * time_interval) / earth_radius) * (180 / math.pi)
    longitude_change = ((velocity_y * time_interval) / earth_radius) * (180 / math.pi) / math.cos(math.radians(latAvg))

    # Update latitude and longitude
    newlatAvg = latAvg + latitude_change
    newlongAvg = longAvg + longitude_change

    print("IMU update")
    print(f"new latitude: {newlatAvg} new longitude: {newlongAvg} velx(m/s): {velocity_x} vely(m/s): {velocity_y}")
    print(f"sensor acceleration (m/s^2): {sensor.linear_acceleration}")

    return newlatAvg, newlongAvg, velocity_x, velocity_y