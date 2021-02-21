import numpy as np
import cv2
import copy
from scipy.stats import trim_mean

def find_color(img,min_hsv,max_hsv):
    '''Identify pixels in image within a color range.'''
    #Convert img to HSV
    img=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    region=np.where((img[:,:,0]>=int(min_hsv[0])) & (img[:,:,0]<=int(max_hsv[0])) & 
                    (img[:,:,1]>=int(min_hsv[1])) & (img[:,:,1]<=int(max_hsv[1])) & 
                    (img[:,:,2]>=int(min_hsv[2])) & (img[:,:,2]<=int(max_hsv[2])))
    
    return region

def color_change(img,search_box,min_hsv=[0,0,95],max_hsv=[100,255,255],
                 new_pixel=[255,0,0],centroid_pixel=[255,0,255]):
    '''Change pixels within a certain color range to a new color.'''
    #Convert input to arrays
    min_hsv=np.array(min_hsv)
    max_hsv=np.array(max_hsv)
    #Isolate desired pixel indices
    region=np.array(find_color(img,min_hsv,max_hsv))
    #Create boolean filter for indexing
    x_filt=np.array((region[1]>=int(search_box['min']['x'])) & (region[1]<=int(search_box['max']['x'])),dtype=bool)
    y_filt=np.array((region[0]>=int(search_box['min']['y'])) & (region[0]<=int(search_box['max']['y'])),dtype=bool)
    region_filter=x_filt & y_filt
    #Build final type of pixel indices
    final_region=(region[0][region_filter],
                  region[1][region_filter])
    #Change color of pixels
    new_img=copy.deepcopy(img)
    new_img[final_region]=new_pixel
    
    # Manual 'Centroid' finding
    try:
        # if len(final_region[0])>200:
        # Identify mean index and mark with circle
        # Mean is trimmed on both tails
        x_ave=int(trim_mean(final_region[1],0.4))
        y_ave=int(trim_mean(final_region[0],0.4))
        center=(x_ave, y_ave)
        cv2.circle(new_img, center, 1, centroid_pixel, 6)
    except ValueError:
        center=(None,None)
        pass
    
    return new_img, center, region

if __name__=='__main__':
    img = cv2.imread('/home/pandarobot/Desktop/key_image.png',1)
    # cv2.imshow('Original',img)
    # print img.shape

    key_search_box={'min':{'x':int(100),'y':int(80)},
                    'max':{'x':int(img.shape[0]-20),'y':int(img.shape[1]-100)}}
    
    img,center,cnt=color_change(img,key_search_box)

    rows,cols = img.shape[:2]
    [vx,vy,x,y] = cv2.fitLine(cnt.T, cv2.DIST_L2,0,0.01,0.01)
    lefty = int((-x*vy/vx) + y)
    righty = int(((cols-x)*vy/vx)+y)
    cv2.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)
    cv2.imshow('With Orientation',img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()