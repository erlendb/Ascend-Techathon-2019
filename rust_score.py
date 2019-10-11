import cv2 as cv
import numpy as np

def rust_score(image_array):
    """
    Function that calculates the rust score for a picture. 

    input: image_array: pixel array in rgb format. Import cv2 and use cv2.imread("filename") then 
    pass it into this function
    return: rust_score, this returns the total number of white pixels found
    """

    #rename input variable
    img = image_array

    #Set hue, value, saturation parameters
    low_hue = 2
    low_sat = 133
    low_val = 19
    upp_hue = 19
    upp_sat = 240
    upp_val = 125

    #rgb image to hue, value saturation image
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #set upper and lower bound for HSV values
    low_bound = np.array([low_hue,low_sat,low_val])
    up_bound = np.array([upp_hue, upp_sat, upp_val])

    #mask
    mask = cv.inRange(hsv_img, low_bound, up_bound)

    #sum white pixels
    n_white_pix = np.sum(mask == 255)

    #calculate rust score
    rust_score = n_white_pix

    return rust_score




