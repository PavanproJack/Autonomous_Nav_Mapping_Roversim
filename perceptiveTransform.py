#!/usr/bin/env python
# coding: utf-8

# # Generate an Image from Test Dataset


import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import glob
import numpy as np



# Path for Test dataset
dataPath = './IMG/*'
# The glob module finds all the pathnames matching a specified pattern according to the rules used by the Unix shell
imgList = glob.glob(dataPath)
# Generate a random integer in-range
randInt = np.random.randint(0, len(imgList)-1)

image = mpimg.imread(imgList[randInt])
plt.imshow(image)





rockImagePath = './calibration_images/rockImage.jpg'
rockSample = mpimg.imread(rockImagePath)

gridImagePath = './calibration_images/example_grid1.jpg'
gridImage = mpimg.imread(gridImagePath)

# figsize is the width and height in inches
fig = plt.figure(figsize=(12,3))

plt.subplot(121) # 121 : row, column, image_number
plt.imshow(gridImage)
plt.subplot(122) # 122 : row, column, image_number
plt.imshow(rockSample)
