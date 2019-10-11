import cv2 as cv
import numpy as np

def crop_base(image_array):
    """
    Denne trengs IKKE å brukes fra API
    rust_score bruker denne
    se nedenfor for å finne rust_score

    """

    #rename input variable
    img = image_array
    R,G,B = cv.split(img)

    #Set hue, value, saturation parameters
    low_hue = 10
    low_sat = 135
    low_val = 0
    upp_hue = 36
    upp_sat = 255
    upp_val = 189

    #to hue, value saturation img
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #lower and upper bound for HSV mask
    low_bound = np.array([low_hue,low_sat,low_val])
    up_bound = np.array([upp_hue, upp_sat, upp_val])

    #HSV mask
    mask = cv.inRange(hsv_img, low_bound, up_bound)

    #get contours
    _, contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    #get max contour
    maxContour = 0
    for contour in contours:
        contourSize = cv.contourArea(contour)
        if contourSize > maxContour:
            maxContour = contourSize
            maxContourData = contour

    # Create a mask from the largest contour
    mask = np.zeros_like(mask)
    cv.fillPoly(mask,[maxContourData],1)

    # Use mask to crop data from original image
    finalImage = np.zeros_like(img)

    finalImage[:,:,0] = np.multiply(R,mask)

    finalImage[:,:,1] = np.multiply(G,mask)

    finalImage[:,:,2] = np.multiply(B,mask)
    
    cv.drawContours(img, contours, 0, (0,0,255), 6)

    return finalImage

def rust_score(image_array):
    """
    Function that calculates the rust score for a picture. 

    input: image_array: pixel array in rgb format. Import cv2 and use cv2.imread("filename") then 
    pass it into this function
    return: rust_score, this returns the total number of white pixels found
    """

    #rename input variable
    img = image_array

    #crop image, get closest windmill base
    img = crop_base(img)


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