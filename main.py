import time
import busio
import board
import adafruit_bno055
import math
import machine
import select

from initialize import initialize_lcd, initialize_gps
from track import get_latitude, get_longitude, get_gps_location, imu_update
from boundary import is_within_polygon

'''
Set of (x,y) coordinates
'''
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def dataReceive():
    print("listening...")
    with open("coordinates.csv", 'r') as f:
        lines = f.readlines()
        lines = [item.replace("\n", "") for item in lines]
        value = ",".join(lines)
        print(value)

    # Temporary Geofence For testing purposes - Use computer-end-code.py for real Geofence Values
    #value = "OUTER, 40.39123253146508, -86.82833099365233, 40.365601883498044, -86.97458648681639, 40.42417189314546, -87.00067901611327, 40.468065962237894, -86.85236358642577, 40.428353515662465, -86.79743194580077, INNER"

    coordinateList = value.split(",")
    innerBegin = coordinateList.index("INNER")

    outerList = [float(coordinateList[idx]) for idx in range(1, innerBegin)]
    innerList = [float(coordinateList[idx]) for idx in range(innerBegin + 1, len(coordinateList))]

    outerList = [(outerList[i], outerList[i+1]) for i in range(0, len(outerList)-1,2)] # Groups the latitudes and longitudes together
    innerList = [(innerList[i], innerList[i+1]) for i in range(0, len(innerList)-1,2)] # Groups the latitudes and longitudes together

    return outerList, innerList

if __name__ == '__main__':
    outerPolygon, innerPolygon = dataReceive()

    i2c = busio.I2C(board.GP15, board.GP14, frequency=1000)       # Initializes I2C for the IMU
    sensor = adafruit_bno055.BNO055_I2C(i2c)                        # Initializes IMU

    gps_uart = initialize_gps()                                   # Initializes GPS
    lcd_uart = initialize_lcd(backlight_red=255, backlight_green=1, backlight_blue=255)
    
    lcd_uart.write(b'|')  # Setting character
    lcd_uart.write(b'-')  # Clear display # Clear Display
    lcd_uart.write(b"Connecting to GPS...")  # For 16x2 LCD
    #time.sleep(1.5) - Can add back in to display message for readability on LCD screen. The GPS sensor needs a few seconds to connect usually anyways. 
    
    relay_on = machine.Pin(11, mode=machine.Pin.OUT) # output for relay on/off
    relay_on.value(1)
    reset_kart = machine.Pin(9, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP) # input for reset or not reset
    
    with open("imu_data.txt", "a") as file:
        print("wiped imu_data.txt\n")
    
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
    
    gps_start_time, imu_start_time = time.ticks_ms(), time.ticks_ms()
    
    latitude_avg,longitude_avg = 0,0
    latitude_avg,longitude_avg = get_gps_location(gps_uart, lcd_uart,gps_start_time)
    initial_time = time.ticks_ms()

    lcd_uart.write(b'|')  # Setting character
    lcd_uart.write(b'-')  # Clear display # Clear Display
    lcd_uart.write(b"IN")  # For 16x2 LCD

    #Main Loop
    while True:
        if reset_kart.value() == 0:
            relay_on.value(0)  # disable kart
            # print("Disabling kart for reset")
            lcd_uart.write(b'|')  # Setting character
            lcd_uart.write(b'-')  # Clear display # Clear Display
            lcd_uart.write(b"Disabling Kart for Reset")  # For 16x2 LCD
            time.sleep(3) # wait for kart to stop
            # print("Resetting system")
            lcd_uart.write(b'|')  # Setting character
            lcd_uart.write(b'-')  # Clear display # Clear Display
            lcd_uart.write(b"Resetting System")  # For 16x2 LCD
            time.sleep(1)
            lcd_uart.write(b'|')  # Setting character
            lcd_uart.write(b'-')  # Clear display # Clear Display
            lcd_uart.write(b"Ensure velocity is 0")  # For 16x2 LCD
            time.sleep(5)
            # regets GPS lock
            latitude_avg,longitude_avg = 0,0
            latitude_avg,longitude_avg = get_gps_location(gps_uart, lcd_uart, time.ticks_ms())
            gps_start_time, imu_start_time = time.ticks_ms(), time.ticks_ms()
            print(f"Initial GPS Lock: {latitude_avg}, {longitude_avg}")
            lcd_uart.write(b'|')  # Setting character
            lcd_uart.write(b'-')  # Clear display # Clear Display
            lcd_uart.write(b"Initial GPS Lock Acquired")  # For 16x2 LCD
            # resets velocity for IMU
            velocity_x = 0
            velocity_y = 0
            time.sleep(2)
            # reenables kart
            relay_on.value(1)
            # print(f"Kart Enabled")
            lcd_uart.write(b'|')  # Setting character
            lcd_uart.write(b'-')  # Clear display # Clear Display
            lcd_uart.write(b"Kart Enabled")  # For 16x2 LCD
            time.sleep(2)
            lcd_uart.write(b'|')  # Setting character
            lcd_uart.write(b'-')  # Clear display # Clear Display
            lcd_uart.write(b"IN")  # For 16x2 LCD

        if relay_on.value() == 1:
            imu_start_time = time.ticks_ms()
            
            #Check if GPS has position:
            str_array = gps_uart.readline()
            if not str_array:
                pass
            else:
                try:
                    has_coords = False
                    str_array = str_array.decode("utf-8").strip().split(",")      # Decodes GPS input
                    if str_array[0] == '$GPGLL':
                        new_latitude_avg = get_latitude(str_array, 1)
                        new_longitude_avg = get_longitude(str_array, 3)
                        has_coords = True
                    elif str_array[0] == '$GPGGA':
                        new_latitude_avg = get_latitude(str_array, 2)
                        new_longitude_avg = get_longitude(str_array, 4)
                        has_coords = True
                    if has_coords and (math.fabs(new_latitude_avg - latitude_avg) < 0.05 and abs(new_longitude_avg - longitude_avg) < 0.05):
                        latitude_avg = new_latitude_avg
                            
                    with open("gps_data.txt", "a") as file:
                        file.write(f"{latitude_avg:.10f},{longitude_avg:.10f},{time.ticks_ms()-gps_start_time}\n")
                    gps_start_time = time.ticks_ms()

                except (ValueError, IndexError):
                    lcd_uart.write(b'|')  # Setting character
                    lcd_uart.write(b'-')  # Clear display # Clear Display
                    lcd_uart.write(b"Error No Signal")  # For 16x2 LCD
                     # print("valueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire")
            
            update_time = time.ticks_ms() - imu_start_time
            latitude_avg, longitude_avg, velocity_x, velocity_y = imu_update(latitude_avg, longitude_avg, update_time, velocity_x, velocity_y, sensor)
            print(f'''IMU update time: {update_time} ms \nIMU refresh rate: {1000 / update_time} Hz''')
            
            if is_within_polygon(outerPolygon, (float(latitude_avg), float(longitude_avg))) is True and is_within_polygon(
                    innerPolygon, (float(latitude_avg), float(longitude_avg))) is False:
                '''
                print("\nKart is in bounds\n")
                '''

            else:
                relay_on.value(0)
                lcd_uart.write(b'|')  # Setting character
                lcd_uart.write(b'-')  # Clear display # Clear Display
                lcd_uart.write(b"OUT")  # For 16x2 LCD
                print(f"Stop distance: {(update_time) / 1000 * (velocity_x ** 2 + velocity_y ** 2) ** 0.5}m")
                '''
                print("\nKart is out of bounds\n")
                print(f"\n\ntime to glitch: {(time.ticks_ms()-initial_time)/1000} secs")
                '''