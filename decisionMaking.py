import numpy as np 

def identifySamples(Rover):
     
    if(Rover.near_sample):
        print("I am near the sample")
        #Rover.brake = 5
        Rover.send_pickup = True
    else:
        print("away")

    return Rover