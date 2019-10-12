import cv2 as cv

def save_photos(windmill_position, photo_id, img, score):
    filename = 'rustimg-'+str(windmill_position.x)+'-'+str(windmill_position.y)+'-'+str(photo_id)+'-'+str(score)+'.jpg'
    print(filename)
    cv.imwrite(filename, img)
