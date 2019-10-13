import cv2 as cv
import numpy as np
#import matplotlib.pyplot as plt

def crop_base(image_array):
    """
    Denne trengs IKKE aa brukes fra API
    rust_score bruker denne
    se nedenfor for aa finne rust_score

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

    #quick fix for aa sette maxCountorData
    '''
    maxContourData = 0
    for contor in contours:
        maxContourData = contor
    '''

    #get max contour
    maxContour = 0
    flag = False
    for contour in contours:
        contourSize = cv.contourArea(contour)
        if contourSize > maxContour:
            maxContour = contourSize
            maxContourData = contour
            flag = True

    if flag:
        # Create a mask from the largest contour
        mask = np.zeros_like(mask)
        cv.fillPoly(mask,[maxContourData],1)

        # Use mask to crop data from original image
        finalImage = np.zeros_like(img)

        finalImage[:,:,0] = np.multiply(R,mask)

        finalImage[:,:,1] = np.multiply(G,mask)

        finalImage[:,:,2] = np.multiply(B,mask)
    else:
        finalImage = img



    return finalImage

def getCentreLocation(cropped_img):
    gray_image = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
    _,finalImage = cv.threshold(gray_image,0,254, cv.THRESH_BINARY)

    M = cv.moments(finalImage)

    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return (cX, cY)


def getWidthofRow(cropped_img, rowNum):
    """
    get width of yellow base
    """

    gray_image = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
    _,treshImage = cv.threshold(gray_image,0,255, cv.THRESH_BINARY)

    mid_row = treshImage[rowNum]

    n_white_pix = np.sum(mid_row == 255)

    return n_white_pix

def getHeightOfBase(cropped_img, cX):
    gray_image = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
    _,treshImage = cv.threshold(gray_image,0,255, cv.THRESH_BINARY)

    transp_treshImage = np.transpose(treshImage)

    mid_col = transp_treshImage[cX]
    n_white_pix = np.sum(mid_col == 255)
    return n_white_pix

def weighing_vertical(y, centreY, height_base, max_weight):
    max_weight = float(max_weight)
    height_base = float(height_base)

    MOVE_CENTRE_Y = 120

    centreY = centreY + MOVE_CENTRE_Y

    #if y coordinate is below centre, dont weigh
    if(y > centreY):
        return 1.0

    if(height_base < 400):
        height_base = 400
        #print("unexptected heigh of base, weighing vertical")

    relative_y = centreY - y
    relative_y = float(relative_y)

    rate_of_change = max_weight / (height_base/2)

    weighing = 1.0 + relative_y*rate_of_change

    if(weighing > max_weight):
        weighing = max_weight

    return weighing




def weighing_horizontal(x, centreX, width, max_weight):

    x = float(x)
    centreX = float(centreX)
    width = float(width)
    x_local = x - centreX
    if(abs(x_local) < 0.001):
        return 1.0

    if(x_local*2 == width):
        return 1.0

    if(x_local*2 > width):
        #print("error in weighing func, x coordinate out of range")
        return 1.0


    x_local = abs(x_local)
    r = width/2

    if x_local/r >= 1.0:
        return max_weight

    theta = np.arccos(x_local/r)
    weight = 1/(np.sin(theta))

    if(weight > max_weight):
        weight = max_weight

    return weight

def addWeighing(rust_mask, centre, cropped_img):
    cX, cY = centre

    TRESHOLD_WIDTH_MAX = 250
    TRESHOLD_WIDTH_MIN = 150
    MAX_HORIZONTAL_WEIGHT = 6 #4 for 26. 5 for 26
    MAX_VERTICAL_WEIGHT = 2.2


    num_rows = len(rust_mask)
    num_coloumns = len(rust_mask[0])

    height_base = getHeightOfBase(cropped_img, cX)

    score = 0
    for row_index in range(num_rows):
        for col_index in range(num_coloumns):
            if rust_mask[row_index][col_index] == 255:
                width_row = getWidthofRow(cropped_img, row_index)
                if(width_row > TRESHOLD_WIDTH_MAX):
                    width_row = TRESHOLD_WIDTH_MAX
                if(width_row < TRESHOLD_WIDTH_MIN):
                    width_row = TRESHOLD_WIDTH_MIN

                weighing_hor = weighing_horizontal(col_index, cX, width_row, MAX_HORIZONTAL_WEIGHT)
                weighing_vert = weighing_vertical(row_index, cY, height_base, MAX_VERTICAL_WEIGHT)
                total_weight = weighing_vert * weighing_hor

                #print("weighing: ", weighing)
                score = score + total_weight

    return score








def rust_score(image_array, weighing=True):
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
    low_hue = 4
    low_sat = 133
    low_val = 10
    upp_hue = 18
    upp_sat = 255
    upp_val = 100

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

    #testing weighing below this
    centre = getCentreLocation(img)
    if weighing:
        final_score_with_weight = addWeighing(mask, centre, img)
    else:
        final_score_with_weight = rust_score

    return final_score_with_weight
