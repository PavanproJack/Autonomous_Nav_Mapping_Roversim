import numpy as np
import cv2
import matplotlib.image as mpimage

bottom_offset = 6
dst_size = 5

worldMapPath = './calibration_images/map_bw.png'
mapImage = mpimage.imread(worldMapPath)

rgb_thresh = (160, 160, 160)

def rockThreshold(img, rock_thresh = (100, 100, 50)):
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rock_thresh[0]) \
                & (img[:,:,1] > rock_thresh[1]) \
                & (img[:,:,2] < rock_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

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


def transformRotation(yawAngle, nZeroPixels):
    yawRadians = np.radians(yawAngle)
    x_rot = nZeroPixels[0] * np.cos(yawRadians) - nZeroPixels[1] * np.sin(yawRadians)
    y_rot = nZeroPixels[0] * np.sin(yawRadians) + nZeroPixels[1] * np.cos(yawRadians)
    return x_rot, y_rot

def transformTranslation(x_rot, y_rot, scale, x_pos, y_pos):
    x_trans = x_pos + x_rot/scale
    y_trans = y_pos + y_rot/scale
    return x_trans, y_trans 

def transformWorld(nZeroPixels, scale, xpos, ypos, yawAngle, world_size):
    
    xRot, yRot = transformRotation(yawAngle, nZeroPixels)
    xTrans, yTrans = transformTranslation(xRot, yRot, scale, xpos, ypos)
    
    x_world = np.clip(np.int_(xTrans), 0, world_size - 1)
    y_world = np.clip(np.int_(yTrans), 0, world_size - 1)
    
    return x_world, y_world


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
    
    #Identify Rock Pixels
    rock_thr_image = rockThreshold(warpedImage)
    Rover.vision_image[:,:,1] = rock_thr_image * 255

    #Rover centric pixels of Rock Samples
    rock_rover_x, rock_rover_y = roverView(rock_thr_image)

    #Rover centric pixels of non-navigable terrain (obstacles)
    rover_x_obs, rover_y_obs = roverView(non_nav_terrain_img)

    #Rover centric pixels of navigable terrain
    rover_x, rover_y = roverView(nav_terrain_img) 
    polarData = cvtPolar(rover_x, rover_y)
    #Calculate the mean of all the directions to generate a 
    # promising navigable direction.
    Rover.nav_angles = polarData[1]

    Rover.nav_dists = polarData[0]

    #Creating World Map View here...
    Rover.worldmap[:, :, 1] = mapImage * 255

    #Rover's pose w.r.t world map is given by Rover.pos and Rover.yaw measures.
    world_rover_pos_x = int(Rover.pos[0])
    world_rover_pos_y = int(Rover.pos[1])
    world_rover_o = Rover.yaw

    Rover.worldmap[world_rover_pos_x, world_rover_pos_y, :] =  255
    print(Rover.pos)
    
    world_size = np.shape(Rover.worldmap)[0]

    #World centric pixels of navigable terrain
    world_x, world_y = transformWorld((rover_x, rover_y), Rover.scale, world_rover_pos_x, world_rover_pos_y, world_rover_o, world_size)
    #Rover.worldmap[world_y, world_x, 2] += 10

    # World Centric pixels of Obstacles
    world_x_obs, world_y_obs = transformWorld((rover_x_obs, rover_y_obs), Rover.scale, world_rover_pos_x, world_rover_pos_y, world_rover_o, world_size)
    #Rover.worldmap[world_y_obs, world_x_obs, 0] += 10









    return Rover
