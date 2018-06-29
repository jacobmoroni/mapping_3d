import numpy as np
import cv2
from ipdb import set_trace
map_raw = cv2.imread('lab_map.png',0)
# cv2.imshow('raw',map_raw)
thresh = 10
map_bw = cv2.threshold(map_raw, thresh, 255, cv2.THRESH_BINARY)[1]
map_bw = cv2.bitwise_not(map_bw)
# cv2.imshow ('bw',map_bw)

#try to clean up noise in the map
kernel = np.ones((5,5),np.uint8)
# map_bw = cv2.dilate(map_bw,kernel,iterations = 3)
# map_bw = cv2.erode(map_bw,kernel,iterations = 3)
map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)
map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)
# map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_OPEN,kernel)
# cv2.imshow('bw_d',map_bw)


#convert to array to begin generating obstacles
map_mat = np.array(map_bw)
obs = np.nonzero(map_mat)
obs = np.array(obs)

#prune obstacles
#TODO add something to pass in less obstacles 

#convert pixels to meters
px_conv=0.03103
ob_list = []
prox_thresh = 0.1
for i,x in enumerate(obs.T):
    x = x*px_conv
    if i = 0:
        ob_list.append((x[0],x[1],0.032))
    else:
        for i in range(0,len(ob_list)):
            if np.sqrt((x[0]-ob_list[i][0])**2+(x[1]-ob_list[i][1])**2)>prox_thresh
            #TODO figure out how to do this loop to only add the obstacle if 
            #no other obstacle already added is within the threshold


# print ob_list


# #shut down when done
# k = cv2.waitKey(0)
# if k == 27:         # wait for ESC key to exit
    # cv2.destroyAllWindows()
# elif k == ord('s'): # wait for 's' key to save and exit
    # cv2.imwrite('messigray.png',img)
    # cv2.destroyAllWindows()
