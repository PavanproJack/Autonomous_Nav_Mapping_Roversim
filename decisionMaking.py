import numpy as np 

def identifySamples(Rover):

    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    print(Rover.nav_angles)
    Rover.throttle = Rover.throttle_set
    if(Rover.near_sample):
        print("I am near the sample")
        Rover.brake = Rover.brake_set
        if Rover.vel == 0 and not Rover.picking_up:
            Rover.send_pickup = True

    return Rover