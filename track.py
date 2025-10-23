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
    if str_array[index + 1] == "S":
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
    if str_array[index2 + 1] == "W":
        longitude = -longitude
    return longitude


# Gets Current Location
def get_gps_location(gps_uart, lcd_uart):
    latitude_LL = 0
    longitude_LL = 0
    latitude_GA = 0
    longitude_GA = 0
    latDivisor = 1
    lonDivisor = 1

    while ((latitude_LL == 0 and longitude_LL == 0) and (latitude_GA == 0 and longitude_GA == 0)):

        time.sleep(0.03)
        str_array = gps_uart.readline()
        # print(str_array)

        if str_array is None:
            continue
        try:

            str_array = str_array.decode("utf-8")  # Decodes GPS input
            time.sleep(0.03)
            str_array = str_array.split(",")
            # print(str_array)                            # Prints GPS Output

            if str_array[0] == '$GPGLL':
                # print("in GPGLL")
                # lcd_uart.write("in GNGLL")
                latitude_LL = get_latitude(str_array, 1)
                longitude_LL = get_longitude(str_array, 3)
                # print("in GPGLL2: Latitude: ", latitude + "  Longitude: ", longitude)
                # lcd_uart.write("in GNGLL2")

            elif str_array[0] == '$GPGGA':
                # print("in GPGGA")
                # lcd_uart.write("in GNGGA")
                latitude_GA = get_latitude(str_array, 2)
                longitude_GA = get_longitude(str_array, 4)
                # print("in GPGGA2: Latitude: ", latitude  + "  Longitude: ", longitude)
        except (ValueError, IndexError):
            lcd_uart.write(b"Error                           ")  # For 16x2 LCD
            print("valueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire")

    if (latitude_LL != 0 and latitude_GA != 0):
        latDivisor = 2

    if (longitude_LL != 0 and longitude_GA != 0):
        lonDivisor = 2

    latitude_avg = (float(latitude_LL) + float(latitude_GA)) / latDivisor
    longitude_avg = (float(longitude_LL) + float(longitude_GA)) / lonDivisor

    # print("LatIN: " + str(latitude_avg) + " LongIN: " + str(longitude_avg))
    return latitude_avg, longitude_avg


def imu_update(latAvg, longAvg, time_interval, velocity_x, velocity_y):
    earth_radius = 6378137.0  # Earth's equitorial radius in meters

    imu_acceleration_x, imu_acceleration_y, imu_acceleration_z = sensor.linear_acceleration

    # print("Accelerations")
    # print(f"Acceleration X: {imu_acceleration_x:.10f}   Acceleration Y: {imu_acceleration_y:.10f}")
    # print("Sensor Linear Accelearion")
    # print(sensor.linear_acceleration)

    # Velocity Estimation
    velocity_x += imu_acceleration_x * time_interval
    velocity_y += imu_acceleration_y * time_interval

    # print("VELOCITIES")
    # print(f"Velocity X: {velocity_x:.10f}   Velocity Y: {velocity_y:.10f}")

    # Position Estimation
    latitude_change = ((velocity_x * time_interval) / earth_radius) * (180 / math.pi)
    longitude_change = ((velocity_y * time_interval) / earth_radius) * (180 / math.pi) / math.cos(math.radians(latAvg))

    # print("LAT AVG/LONGAVG")
    # print(f"Latitude: {latAvg:.10f}   Longitude: {longAvg:.10f}")

    # print("CHANGES")
    # print(f"Latitude Change: {latitude_change:.10f}   Longitude Change: {longitude_change:.10f}")

    # Update latitude and longitude
    newlatAvg = latAvg + latitude_change
    newlongAvg = longAvg + longitude_change

    # logging into text file
    # TODO: test

    with open(updateFile, "a") as file:
        file.write("IMU UPDATE\n\n")
        file.write("Latitude: {newlatAvg:.10f}   Longitude: {newlongAvg:.10f}\n\n")
        file.write("IMU DATA: {sensor.linear_acceleration}\n\n")

    # print("IMU Refresh Rate: ", float(endTime - startTime))
    print("IMU update")
    print(f"new latitude: {newlatAvg} new longitude: {newlongAvg} velx(m/s): {velocity_x} vely(m/s): {velocity_y}")
    print(f"sensor acceleration (m/s^2): {sensor.linear_acceleration}")

    return newlatAvg, newlongAvg, velocity_x, velocity_y
