
import time
import busio
import board
import adafruit_bno055
import machine
import math

from tracking import Point, is_within_polygon, get_gps_location, get_latitude, get_longitude, initialize_gps, initialize_lcd, imu_update
from boundary import getOuterBoundary, getInnerBoundary

if __name__ == '__main__':

    outerPolygon = []
    innerPolygon = []

    i2c = busio.I2C(board.GP15, board.GP14, frequency=1000)                                   # Initializes I2C for the IMU
    sensor = adafruit_bno055.BNO055_I2C(i2c)                                                  # Initializes IMU
    measure_outer = machine.Pin(board.GP6, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)     # outer boundary switch
    measure_inner = machine.Pin(board.GP7, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)     # inner boundary switch
    kart_in = machine.Pin(board.GP8, mode=machine.Pin.OUT)                                    # relay signal
    kart_in.value(1)                                                                          # start with kart enabled
    reset = machine.Pin(board.GP9, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)             # reset switch
    power_off = machine.Pin(board.GP10, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)        # power off switch

    gps_uart = initialize_gps()                                   # Initializes GPS
    lcd_uart = initialize_lcd(backlight_red=255, backlight_green=1, backlight_blue=255)
    
    lcd_uart.write(b"Connecting to GPS...            ")  # For 16x2 LCD
    #time.sleep(1.5) - Can add back in to display message for readability on LCD screen. The GPS sensor needs a few seconds to connect usually anyways. 
    
    # Example polygon for testing
    '''
    outerPolygon = [
    (40.430484, 86.915721),
    (40.430454, 86.915769),
    (40.430806, 86.916144),
    (40.430835, 86.916097)
    ]
    
    innerPolygon = [
    (40.430484, 86.915721),
    (40.430454, 86.915769),
    (40.430806, 86.916144),
    (40.430835, 86.916097)
    ]
    '''

    #Initalize the GPS position and time trackers
    velocity_x = 0
    velocity_y = 0
    
    latitude_avg,longitude_avg = 0,0
    latitude_avg,longitude_avg = get_gps_location(gps_uart)
    gps_start_time, imu_start_time = time.ticks_ms(), time.ticks_ms()
    print(f"Initial GPS Lock: {latitude_avg}, {longitude_avg}")
    lcd_uart.write(b"Initial GPS Lock Acquired      ")  # For 16x2 LCD
    time.sleep(2)

    outerPolygon = getOuterBoundary(outerPolygon, measure_outer, lcd_uart, gps_uart)
    time.sleep(0.5)
    innerPolygon = getInnerBoundary(innerPolygon, measure_inner, lcd_uart, gps_uart)
    time.sleep(5)

    #Main Loop
    while True:
        if power_off.value() == 0:
            kart_in.value(0)  # disable kart
            # reset boundary files
            print("Resetting boundary")
            lcd_uart.write(b"Resetting Boundary            ")  # For 16x2 LCD
            with open("outer_boundary.txt", "w") as file:
                file.write("")
            with open("inner_boundary.txt", "w") as file:
                file.write("")
            time.sleep(2)
            print("Powering off system")
            lcd_uart.write(b"Powering Off System            ")  # For 16x2 LCD
            break
        if reset.value() == 0:
            kart_in.value(0)  # disable kart
            print("Disabling kart for reset")
            lcd_uart.write(b"Disabling Kart for Reset       ")  # For 16x2 LCD
            time.sleep(3) # wait for kart to stop
            print("Resetting system")
            lcd_uart.write(b"Resetting System               ")  # For 16x2 LCD
            # regets GPS lock
            latitude_avg,longitude_avg = 0,0
            latitude_avg,longitude_avg = get_gps_location(gps_uart)
            gps_start_time, imu_start_time = time.ticks_ms(), time.ticks_ms()
            print(f"Initial GPS Lock: {latitude_avg}, {longitude_avg}")
            lcd_uart.write(b"Initial GPS Lock Acquired      ")  # For 16x2 LCD
            # resets velocity for IMU
            velocity_x = 0
            velocity_y = 0
            time.sleep(2)
            # reenables kart
            kart_in.value(1)
            print(f"Kart Enabled")
            lcd_uart.write(b"Kart Enabled                   ")  # For 16x2 LCD
        
        if kart_in.value() == 1:
            latitude_LL = longitude_LL = latitude_GA = longitude_GA = 0
            latDivisor = lonDivisor = 1
            
            #Check if GPS has position:
            str_array = gps_uart.readline()
            if not str_array:
                pass
            else:
                try:
                    str_array = str_array.decode("utf-8").strip().split(",")      # Decodes GPS input
                    if str_array[0] == '$GPGLL':
                        latitude_LL = get_latitude(str_array, 1)
                        longitude_LL = get_longitude(str_array, 3)
                    elif str_array[0] == '$GPGGA':
                        latitude_GA = get_latitude(str_array, 2)
                        longitude_GA = get_longitude(str_array, 4)
                    if (latitude_LL is not None or latitude_GA is not None) and (longitude_LL is not None or longitude_GA is not None):
                        lat_values = [v for v in (latitude_LL, latitude_GA) if v is not None]
                        lon_values = [v for v in (longitude_LL, longitude_GA) if v is not None]
                        latitude_avg = sum(lat_values) / len(lat_values)
                        longitude_avg = sum(lon_values) / len(lon_values)
                    
                        print(f"GPS UPDATE\nLatitude: {latitude_avg:.10f}\nLongitude: {longitude_avg:.10f}\nRaw Data: {str_array}\nGPS UPDATE TIME: {time.ticks_ms()-gps_start_time}ms\n")
                        gps_start_time = time.ticks_ms()
                except (ValueError, IndexError):
                    lcd_uart.write(b"Error No Signal                 ")  # For 16x2 LCD
                    print("valueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire")
            
            if is_within_polygon(outerPolygon, (float(latitude_avg), float(longitude_avg))) is True and is_within_polygon(
                    innerPolygon, (float(latitude_avg), float(longitude_avg))) is False:
                lcd_uart.write(b"IN                              ")  # For 16x2 LCD
                print("\nKart is in bounds\n")
            else:
                kart_in.value(0)
                signal_time = time.ticks_ms() - imu_start_time
                with open("stopping_time.txt", "w") as file:
                    file.write(f"Processing Time: {signal_time}\n")
                    vel = math.sqrt(velocity_x**2 + velocity_y**2)
                    file.write(f"Magnitude of Velocity upon Exit: {vel}\n")
                    file.write(f"Processing Distance: {(vel*signal_time)/1000} meters\n")
                lcd_uart.write(b"OUT                             ")  # For 16x2 LCD
                print("\nKart is out of bounds\n")

            imu_start_time = time.ticks_ms()
            latitude_avg, longitude_avg, velocity_x, velocity_y = imu_update(latitude_avg, longitude_avg, (time.ticks_ms()-imu_start_time)/10000, velocity_x, velocity_y, sensor)
            update_time = time.ticks_ms() - imu_start_time
            print(f'''IMU update time: {update_time} ms \nIMU refresh rate: {1000 / update_time} Hz''')

            # log data with col 0 = latitude, col 1 = longitude, separate with ;
            with open("coord_datalog.txt", "a") as file:
                file.write(f"{latitude_avg:.10f};{longitude_avg:.10f}\n")
            # log data with col 0 = x velocity, col 1 = y velocity, separate with ;
            with open("velocity_datalog.txt", "a") as file:
                file.write(f"{velocity_x:.10f};{velocity_y:.10f}\n")