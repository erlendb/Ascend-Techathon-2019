import cv2 as cv
import numpy as np

def is_rust(image_array, pixel_treshold):
    """
    Function that checks whetever rust is present in the picture. 
    
    Input: image_array: RGB 3 channel image. Import cv2 and load with cv2.imread("filename")
    Input: pixel_treshold: the treshold for number of white pixel present for the function to return true
    Return: True or false if enough rust is found
    """
    
    #Set hue, value, saturation parameters
    low_hue = 2
    low_sat = 133
    low_val = 19
    upp_hue = 19
    upp_sat = 230
    upp_val = 125

    #Read image
    img = image_array

    #To Hue, value saturation img
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #Create upper and lower bound for HSV range
    low_bound = np.array([low_hue,low_sat,low_val])
    up_bound = np.array([upp_hue, upp_sat, upp_val])

    #create black and white mask, 1 channel array
    mask = cv.inRange(hsv_img, low_bound, up_bound)

    #sum white pixels
    n_white_pix = np.sum(mask == 255)

    #Check if number of white pixels found is above or below limit
    if(n_white_pix > pixel_treshold):
        return True
    else:
        return False