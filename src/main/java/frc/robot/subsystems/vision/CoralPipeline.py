import cv2
import numpy as np

LOWER = (0, 0, 30)
UPPER = (180, 50, 255)

def drawDecorations(image):
    cv2.putText(image, 
        'Limelight python !', 
        (0, 230), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        .5, (0, 255, 0), 1, cv2.LINE_AA)
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, LOWER, UPPER)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    largestContour = np.array([[]])
    
    llpython = [0,0,0,0,0,0,0,0]

    drawDecorations(image)

    contours, _ = cv2.findContours(mask, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, 255, 2)
        largestContour = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largestContour)
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        llpython = [1,x,y,w,h,9,8,7]  
  
    return largestContour, image, llpython
