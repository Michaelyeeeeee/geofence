import time
from tracking import get_gps_location

def getOuterBoundary(outerPolygon, signal, lcd_uart, gps_uart):
    # prompt to start measuring outer boundary
    print("Press the outer boundary switch to start measuring the outer boundary")
    lcd_uart.write(b"Press Outer Switch to Start    ")  # For 16x2 LCD

    # wait for outer switch to flip on
    while signal.value() == 1:
        pass
    # measure points until switch is flipped off
    print("Starting outer boundary measurement")
    lcd_uart.write(b"Measuring Outer Boundary       ")  # For 16x2 LCD
    while signal.value() == 0:
        latitude_avg,longitude_avg = get_gps_location(gps_uart)
        print(f"New Outer Boundary Point: {latitude_avg}, {longitude_avg}")
        lcd_uart.write(b"New Outer Boundary Point       ")  # For 16x2 LCD
        time.sleep(0.25)
        lcd_uart.write(b"Measuring Outer Boundary       ")  # For 16x2 LCD
        outerPolygon.append((latitude_avg,longitude_avg))
        with open("outer_boundary.txt", "a") as file:
            file.write(f"{latitude_avg}, {longitude_avg}\n")
    print("Finished outer boundary measurement")
    lcd_uart.write(b"Finished Outer Boundary        ")  # For 16x2 LCD

    return outerPolygon

def getInnerBoundary(innerPolygon, signal, lcd_uart, gps_uart):
    # pause before inner boundary measurement
    print("Press the inner boundary switch to start measuring the inner boundary")
    lcd_uart.write(b"Press Inner Switch to Start    ")  # For 16x2 LCD

    # wait for inner switch to flip on
    while signal.value() == 1:
        pass
    # measure points until switch is flipped off
    print("Starting inner boundary measurement")    
    lcd_uart.write(b"Measuring Inner Boundary       ")  # For 16x2 LCD
    while signal.value() == 0:
        latitude_avg,longitude_avg = get_gps_location(gps_uart)
        print(f"New Inner Boundary Point: {latitude_avg}, {longitude_avg}")
        lcd_uart.write(b"New Inner Boundary Point       ")  # For 16x2 LCD
        time.sleep(0.25)
        lcd_uart.write(b"Measuring Inner Boundary       ")  # For 16x2 LCD
        innerPolygon.append((latitude_avg,longitude_avg))
        with open("inner_boundary.txt", "a") as file:
            file.write(f"{latitude_avg}, {longitude_avg}\n")
    print("Finished inner boundary measurement")
    lcd_uart.write(b"Finished Inner Boundary        ")  # For 16x2 LCD

    return innerPolygon