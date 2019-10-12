'''
import cv2 as cv

def save_photos(windmill_position, photo_id, img, score):
    filename = 'rustimg-'+str(windmill_position.x)+'-'+str(windmill_position.y)+'-'+str(photo_id)+'-'+str(score)+'.jpg'
    print(filename)
    cv.imwrite(filename, img)
'''

import cv2 as cv

def save_photos(windmill_position, photo_id, img, score, totalScore):
    filename = 'rustimg-'+str(totalScore)+'---'+str(windmill_position.x)+'-'+str(windmill_position.y)+'---'+str(photo_id)+'---'+str(score)+'.jpg'


    x_pos_str = "x_pos: " + str(windmill_position.x)
    y_pos_str = "y_pos: " + str(windmill_position.y)
    id_str = "id: " + str(photo_id)
    score_str = "score: " + str(score)
    tot_score_str = "tot score: " + str(totalScore)


    stringArray = [x_pos_str, y_pos_str, id_str, score_str, tot_score_str]

    font = cv.FONT_HERSHEY_SIMPLEX
    blue = (255,0,0)

    text_start_x = 10
    text_start_y = 40
    font_scale = 1
    thickness = 2


    for string in stringArray:
        text_start = (text_start_x, text_start_y)
        img = cv.putText(img, string, text_start, font, font_scale, blue, thickness)
        text_start_y = text_start_y + 30

    cv.imwrite(filename, img)
