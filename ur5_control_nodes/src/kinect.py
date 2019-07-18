import freenect
import cv2
import numpy as np


i=0
low = np.array([100,50,50]) 
high= np.array([140,255,255]) 
 
#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array

#getting hsvS
def get_hsv():
    tracker = cv2.TrackerKCF_create()
    
    img=get_video()
    bbox = (287, 23, 86, 320)
    bbox = cv2.selectROI(img, False)
    ok = tracker.init(img,bbox)
    
    while True:
        img=freenect.sync_get_video()
        ok,bbox=tracker.update(img)
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(img, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(img, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        
        
        return img




    #hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #mask=cv2.inRange(hsv,low,high)
    #M=cv2.moments(mask)
    #cX = int(M["m10"] / M["m00"])
    #cY = int(M["m01"] / M["m00"])
    #res= cv2.bitwise_and(img,img,mask=mask)
    #cv2.circle(res, (cX, cY), 5, (255, 255, 255), -1)
    
    #return res
    
         
       
 
                
               
 
        # Calculate Frames per second (FPS)
    
 
        # Draw bounding box
        
     
 
    
    
    #hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #mask=cv2.inRange(hsv,low,high)
    #M=cv2.moments(mask)
    #cX = int(M["m10"] / M["m00"])
    #cY = int(M["m01"] / M["m00"])
    #res= cv2.bitwise_and(img,img,mask=mask)
    #cv2.circle(res, (cX, cY), 5, (255, 255, 255), -1)
    
    #return res
 
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array
 
if __name__ == "__main__":
   
    while 1:
        #get a frame from RGB camera
        frame = get_video()
        #get a frame from depth sensor
        depth = get_depth()
        #get a frane from hsv 
        hue=get_hsv()
        #display RGB image
        cv2.imshow('RGB image',frame)
        #display depth image
        cv2.imshow('Depth image',depth)
        #cv2.imshow('Depth image',hue)
        #display hsv
        cv2.imshow('HSV',hue)
 
        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()