import time
import busio
import board
import adafruit_bno055
import math
import machine
import select

from initialize import initialize_lcd, initialize_gps
from track import get_latitude, get_longitude, get_gps_location, imu_update
from boundary import onSegment, orientation, doIntersect, is_within_polygon

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
    
    

    loggingFileName = "test.txt"

    outerPolygon, innerPolygon = dataReceive()

    i2c = busio.I2C(board.GP15, board.GP14, frequency=1000)       # Initializes I2C for the IMU
    sensor = adafruit_bno055.BNO055_I2C(i2c)                        # Initializes IMU

    gps_uart = initialize_gps()                                   # Initializes GPS
    lcd_uart = initialize_lcd(backlight_red=255, backlight_green=1, backlight_blue=255)
    
    lcd_uart.write(b"Connecting to GPS...            ")  # For 16x2 LCD
    #time.sleep(1.5) - Can add back in to display message for readability on LCD screen. The GPS sensor needs a few seconds to connect usually anyways. 
    
    relay_on = machine.Pin(11, mode=machine.Pin.OUT)
    relay_on.value(1)
    
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
    
    # CHANGE IMU SETTINGS HERE
    imu_update_points = 7 # This value can be further optimized. If set to zero, there will be no IMU points (only GPS points).
    imu_time_interval = 0.14 # This value can be further optimized. See IMU BNO055 documentation for minimum refresh rate.
    
    #Initalize the GPS position and time trackers
    velocity_x = 0
    velocity_y = 0
    
    gps_start_time, imu_start_time = time.ticks_ms(), time.ticks_ms()
    
    latitude_avg,longitude_avg = 0,0
    latitude_avg,longitude_avg = get_gps_location(gps_uart, lcd_uart,loggingFileName,gps_start_time)
    initial_time = time.ticks_ms()

    #Main Loop
    while True:
        imu_start_time = time.ticks_ms()
        
        latitude_LL = longitude_LL = latitude_GA = longitude_GA = 0
        latDivisor = lonDivisor = 1
        
        #Check if GPS has position:
        str_array = gps_uart.readline()
        if not str_array:
            pass
        else:
            try:
                str_array = str_array.decode("utf-8").strip().split(",")      # Decodes GPS input
                #print(str_array)                            # Prints GPS Output
                if str_array[0] == '$GPGLL':
                    latitude_LL = get_latitude(str_array, 1)
                    longitude_LL = get_longitude(str_array, 3)
                    #lcd_uart.write("in GNGLL")
                    #print("in GPGLL: Latitude: ", latitude + "  Longitude: ", longitude)

                elif str_array[0] == '$GPGGA':
                    latitude_GA = get_latitude(str_array, 2)
                    longitude_GA = get_longitude(str_array, 4)
                    #lcd_uart.write("in GNGGA")
                    #print("in GPGGA: Latitude: ", latitude  + "  Longitude: ", longitude)
                
                if((latitude_LL or latitude_GA) and (longitude_LL or longitude_GA)):
                    if (latitude_LL and latitude_GA):
                        latDivisor = 2
                    
                    if (longitude_LL and longitude_GA):
                        lonDivisor = 2
                    
                    new_latitude_avg = (latitude_LL + latitude_GA) / latDivisor
                    new_longitude_avg = (longitude_LL + longitude_GA) / lonDivisor
                    if(math.fabs(new_latitude_avg - latitude_avg) < 0.05 and abs(new_longitude_avg - longitude_avg) < 0.05):
                        latitude_avg=new_latitude_avg
                        longitude_avg=new_longitude_avg
                
                    print(f'''
        GPS UPDATE\n
        Latitude: {latitude_avg:.10f}   Longitude: {longitude_avg:.10f}\n
        Raw Data: {str_array}\n
        GPS UPDATE TIME: {time.ticks_ms()-gps_start_time}ms\n
                        ''')
                    
                    #logging into text file 
                    #TODO: test
                        
                    with open(loggingFileName, "a") as file:
                        file.write("GPS UPDATE\n\n")
                        file.write(f"Latitude: {latitude_avg:.10f}   Longitude: {longitude_avg:.10f}\n\n")
                        file.write(f"Raw Data: {str_array}\n\n")
                        file.write(f"GPS UPDATE TIME: {time.ticks_ms()-gps_start_time}ms\n")
                    
                    gps_start_time = time.ticks_ms()
            except (ValueError, IndexError):
                lcd_uart.write(b"Error No Signal                 ")  # For 16x2 LCD
                print("valueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire")
        
        
        update_time = time.ticks_ms() - imu_start_time
        latitude_avg, longitude_avg, velocity_x, velocity_y = imu_update(latitude_avg, longitude_avg, update_time, velocity_x, velocity_y, sensor, loggingFileName)
        print(f'''IMU update time: {update_time} ms \nIMU refresh rate: {1000 / update_time} Hz''')

        if is_within_polygon(outerPolygon, (float(latitude_avg), float(longitude_avg))) is True and is_within_polygon(
                innerPolygon, (float(latitude_avg), float(longitude_avg))) is False:
            lcd_uart.write(b"IN                              ")  # For 16x2 LCD
            print("\nKart is in bounds\n")
        else:
            lcd_uart.write(b"OUT                             ")  # For 16x2 LCD
            print("\nKart is out of bounds\n")
            relay_on.value(0)
            with open(loggingFileName, 'a') as file:
                file.write(f"\n\n\ntime to glitch: {(time.ticks_ms()-initial_time)/1000} secs")
                print(f"\n\ntime to glitch: {(time.ticks_ms()-initial_time)/1000} secs")
            break