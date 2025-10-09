# Preconditions
1. reset switch off
2. power_off switch off

# Preprocess:
1. initialize all peripherals
2. Get initial GPS position
3. Get outer boundary
4. Get inner boundary

# Main loop:
1. Check if power_off switch on - if on then power off kart and reset boundaries
2. Check if reset switch on - if on then power off kart, re-gets kart position/velocity, power on kart
3. If neither switch on and kart in bounds - get GPS, check within bounds, get IMU, log data, repeat

If GPS reports out of bounds, kart_in = 0 and power is cut until reset.

# boundary.py:
getOuterBoundary - user flips on outer_boundary switch to start recording points for boundary, flip off to stop
getInnterBoundary - user flips on inner_boundary switch to start recording points for inner boundary, flip off to stop

# tracking.py:
initialization functions for GPS, IMU, LCD
getLatitude/Longitude functions
check within_polygon functions
gps/imu update functions