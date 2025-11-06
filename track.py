import time
import busio
import board
import adafruit_bno055
import math
import machine
import select


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

# Gets Current Location
def get_gps_location(gps_uart, lcd_uart,gps_start_time):
    latitude_LL = 0
    longitude_LL = 0
    latitude_GA = 0 
    longitude_GA = 0
    latDivisor = 1
    lonDivisor = 1
    
    while ((latitude_LL==0 and longitude_LL==0) and (latitude_GA==0 and longitude_GA==0)):
            
        time.sleep(0.25)
        str_array = gps_uart.readline()
        # print(str_array)
        
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
            lcd_uart.write(b'|')  # Setting character
            lcd_uart.write(b'-')  # Clear display # Clear Display
            lcd_uart.write(b"Attempting to retrieve GPS coords")  # For 16x2 LCD
            print("valueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire")
    
    if (latitude_LL != 0 and latitude_GA != 0):
        latDivisor = 2
    
    if (longitude_LL != 0 and longitude_GA != 0):
        lonDivisor = 2
    
    latitude_avg = (float(latitude_LL) + float(latitude_GA)) / latDivisor
    longitude_avg = (float(longitude_LL) + float(longitude_GA)) / lonDivisor
    
    print("wiped gps_data.txt")
    with open("gps_data.txt", "w") as file:
        file.write(f"latitude, longitude, update time (m/s)\n")
        file.write(f"initial: {latitude_avg:.10f},{longitude_avg:.10f},{time.ticks_ms()-gps_start_time}\n")
    return latitude_avg, longitude_avg





''' @brief Converts quaternion to rotation matrix and rotates vector
    @param q: quaternion (w,x,y,z)
'''
def quat_to_rot_matrix(q):
    if len(q) != 4:
        raise ValueError("Quaternion must have 4 elements")
    # bno055 always returns (w,x,y,z)
    w, x, y, z = q
    # normalize
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm == 0:
        return [[1,0,0],[0,1,0],[0,0,1]]
    w/=norm; x/=norm; y/=norm; z/=norm
    # rotation matrix (body -> nav)
    R = [
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ]
    return R
''' @brief Rotates vector v using rotation matrix R
    @param R: rotation matrix'''
def rotate_vector(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )

# meters-per-degree approximations (WGS-84 based)
def meters_per_degree_lat(lat_rad):
    # latitude in radians
    # approximate length of a degree latitude (meters)
    return 111132.92 - 559.82 * math.cos(2*lat_rad) + 1.175 * math.cos(4*lat_rad) - 0.0023 * math.cos(6*lat_rad)

def meters_per_degree_lon(lat_rad):
    # length of a degree longitude (meters)
    return 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3*lat_rad) + 0.118 * math.cos(5*lat_rad)

''' @brief Updates latitude, longitude, and velocities using IMU data

    @param lat, lon: current latitude, longitude in degrees
    @param dt: time interval in seconds (float)
    @param vel_x, vel_y: velocities in m/s along local north (x) and east (y)
    @param sensor: BNO055 sensor object with linear_acceleration and orientation/quaternion
    @return: new_lat, new_lon, new_vel_x, new_vel_y'''
def imu_update(lat, lon, dt, vel_x, vel_y, sensor):
    if dt <= 0:
        return lat, lon, vel_x, vel_y

    # read accelerometer
    ax_b, ay_b, az_b = sensor.linear_acceleration
    # quaternion
    q = sensor.quaternion

    if q is not None:
        try:
            R = quat_to_rot_matrix(q)
            ax_n, ay_n, az_n = rotate_vector(R, (ax_b, ay_b, az_b))
        except Exception:
            # fallback: assume body==nav
            ax_n, ay_n, az_n = ax_b, ay_b, az_b
    else:
        # no orientation info: assume accelerations are already in nav frame
        ax_n, ay_n, az_n = ax_b, ay_b, az_b

    # integrate acceleration -> velocity using simple Euler/trapezoid:
    new_vel_x = vel_x + ax_n * dt
    new_vel_y = vel_y + ay_n * dt

    # displacement using average velocity (trapezoidal integration)
    disp_n = 0.5 * (vel_x + new_vel_x) * dt   # north displacement in meters
    disp_e = 0.5 * (vel_y + new_vel_y) * dt   # east displacement in meters

    # convert meter displacements to degree changes
    lat_rad = math.radians(lat)
    m_per_deg_lat = meters_per_degree_lat(lat_rad)
    m_per_deg_lon = meters_per_degree_lon(lat_rad)
    delta_lat_deg = disp_n / m_per_deg_lat
    delta_lon_deg = disp_e / m_per_deg_lon

    new_lat = lat + delta_lat_deg
    new_lon = lon + delta_lon_deg

    try:
        with open("imu_data.txt", 'a') as f:
            f.write(f"{new_lat}, {new_lon}, {ax_b}, {ay_b}, {az_b}, {ax_n}, {ay_n}, {az_n}, {vel_x}, {vel_y}, {new_vel_x}, {new_vel_y}, {dt}")
    except Exception:
        pass

    return new_lat, new_lon, new_vel_x, new_vel_y