"""
File to generate and save aruco markers

@author: Robin Amsters
@email: robin.amsters@kuleuven.be

Original file: http://www.philipzucker.com/aruco-in-opencv/

Generates an aruco marker with a size of the specified number of pixels. Paste this image for example in word to fix the physical dimensions in meters (needed for detection later).
"""
import cv2
import cv2.aruco as aruco

# Paramters for marker generation
n_markers = 1 # Number of tags to generate
imageSize = 700 # Size of image in pixels

for i in range(n_markers):
    markerID = i

    # Get dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    
    # Generate and save marker
    img = aruco.drawMarker(aruco_dict, markerID, imageSize) # dictionary, id, image size
    cv2.imwrite("marker_" + str(markerID) + "_pxl_" + str(imageSize) +".png", img) # Save image
    
    # Show marker untill key is pressed
    cv2.imshow('Generated marker',img) 
    cv2.waitKey(0)
    cv2.destroyAllWindows()
