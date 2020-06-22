import numpy as np
import cv2

bottom_offset = 6
dst_size = 5

rgb_thresh = (160, 160, 160)

def roverView(image):
    ypos, xpos = image.nonzero()
    x_pixel = xpos - image.shape[1]/2
    y_pixel = image.shape[0] - ypos    
    return x_pixel, y_pixel

def cvtPolar(rx, ry):
    distance = np.sqrt(rx**2 + ry**2)
    angle = np.arctan2(ry, rx) 
    # Calculate every non-zero pixel's distance and direction(angle) 
    # from the rover's center and return it.

    return distance, angle 


def perspectiveTransform(Rover):
    image = Rover.img
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              [image.shape[1] / 2 - dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              ])
    # dst is the output image that has the size dsize and the same type as src .
    # Returns a 3x3 Transformation Matrix by calculating a perspective transform 
    # from a four pairs of the corresponding points.
    TMatrix = cv2.getPerspectiveTransform(source, destination)
    # dsize is the size of the output image.
    dsize = (image.shape[1], image.shape[0])  # keep same size as input image
    # warpPerspective applies a perspective transformation to an image.
    warpedImage = cv2.warpPerspective(image, TMatrix, dsize)
    maskedImage = cv2.warpPerspective(np.ones_like(image[:, :, 0]), TMatrix, dsize)

    nav_terrain_img = np.zeros_like(warpedImage[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (warpedImage[:, :, 0] > rgb_thresh[0]) \
                   & (warpedImage[:, :, 1] > rgb_thresh[1]) \
                   & (warpedImage[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    nav_terrain_img[above_thresh] = 1


    # Now insert nav_terrain_img into blue channel and non_nav_terrain_img into red channel
    Rover.vision_image[:,:,2] = nav_terrain_img * 255  

    non_nav_terrain_img = np.absolute(np.float32(nav_terrain_img) - 1) * maskedImage
    Rover.vision_image[:,:,0] = non_nav_terrain_img * 255  


    #Rover centric pixels of navigable terrain
    rover_x, rover_y = roverView(nav_terrain_img) 
    polarData = cvtPolar(rover_x, rover_y)
    #Calculate the mean of all the directions to generate a 
    # promising navigable direction.
    Rover.nav_angles = polarData[1]
    Rover.nav_dists = polarData[0]


    return Rover
