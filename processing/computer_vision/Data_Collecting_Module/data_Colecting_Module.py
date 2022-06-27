import os
import cv2
import time 

path = 'data/images'
cameraNumber = 0 # 1 For external camera, 0 for internal
cameraBrightness = 150
moduleValue = 10
minBlur = 150
grayImage = False
saveData = True
showImage = True
imgWidth = 640
imgHeight = 480

global countFolder
capture = cv2.VideoCapture(cameraNumber)
capture.set(3, 640)
capture.set(4, 480)
capture.set(10, cameraBrightness)

count = 0
countSave = 0

def saveDataFunction():
     global countFolder
     countFolder = 0
     while os.path.exists(path + str(countFolder)):
          countFolder += 1
     os.makedirs(path + str(countFolder))

if saveData:saveDataFunction()

while True:

     success, img = capture.read()
     img = cv2.resize(img, (imgWidth, imgHeight))
     if grayImage:img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
     if saveData:
          blur = cv2.Laplacian(img, cv2.CV_64F).var()
          print(blur)
          if count % moduleValue == 0 and blur > minBlur:
               nowTime = time.time() #Why is time.time(), does not make any sense
               cv2.imwrite(path + str(countFolder) + '/' + str(countSave) + " " + str(int(blur)) + " " + str(nowTime) + ".png", img)
               countSave += 1
          count += 1

     if showImage:
          cv2.imshow("Image", img)

     if cv2.waitKey(1) & 0xFF == ord('q'):
          break

capture.release()
cv2.destroyAllWindows()
