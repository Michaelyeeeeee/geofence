'''
Jonas Mukund
EPICS - Eletric Vehicle Event Infrastructure
10/16/2025 

The goal of this program is predict how far the kart will travel after crossing the geofence
 under braking and non-breaking conditions
'''
def main():
    # Initilize basic variables
    GRAVITY = 9.81 # m/s
    frictional_coefficient = 1 # Average for racecar tire and road
    weight = 500 / 2.20462262 # Pounds to Kg, ESTIMATE
    velocity = float(input("How fast is the kart going as it crosses the geofence (mph): "))  
    braking = (input("Will the user be braking (Yes/No): ").strip()[:1].upper() == "Y")
    recognition_time = 1 # Seconds
    # handle breaking scenario
    if braking:
        braking = 3 * GRAVITY * weight # Medium Approximation, 2gs
        
        force = weight * GRAVITY * frictional_coefficient + braking
        acceleration = -force / weight 
        # Assume constant acceleration
        time = -velocity / acceleration 
        distance = velocity * time / 2 + recognition_time * velocity
    
    else:
        force = weight * GRAVITY * frictional_coefficient
        acceleration = -force / weight 
        # Assume constant acceleration
        time = -velocity / acceleration + recognition_time
        # Will travel recognition time at max velocity
        distance = velocity * time / 2 + recognition_time * velocity

    print(f"The kart will stop after {time+recognition_time:.3f} seconds.\nThe cart will travel {distance:.3f} meters past the geofence.")
    # handle friction scenario

if __name__ == "__main__":
    main()
