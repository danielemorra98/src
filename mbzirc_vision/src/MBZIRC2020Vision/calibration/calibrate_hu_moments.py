# coding=utf-8

import cv2
import numpy as np
import time
import math
import sys
import getopt
import socket
from streaming.upstream import Upstreamer

# Streaming
streaming = False   # stream the video to ground station?
addr = '127.0.0.1'  # default IP address to stream
port = 54321        # default server port to stream

try:
    opts, args = getopt.getopt(sys.argv[1:], "hsa:p:", \
                 ['help', 'streaming', 'addr=', 'port='])
except getopt.GetoptError as err:
    print 'Check the README file (or ask Lai) for info about usage.'
    sys.exit(2)
for o, a in opts:
    if o in ("-s", "--streaming"):
        streaming = True
    elif o in ("-a", "--addr"):
        addr = a
        try:
            socket.inet_aton(addr)
        except socket.error:
            assert False, "invalid IP address"
    elif o in ("-p", "--port"):
        port = int(a)
        assert (port > 1024 and port <= 65535), "invalid port"
    elif o in ("-h", "--help"):
        print "Please ask Lai about usage."
        sys.exit()
    else:
        assert False, "unhandled option: " + o



#caratteristiche RUNCAM 2 Risol: 640x480 (fissa per funzionamento come webcam), 1/3'' sensor 4:3
f = 2.8/1000 # focal length [m]
w = 4.8/1000 # sensor width [m]
h = 3.6/1000 # sensor height [m]
diag = 6/1000 # sensor diagonal [m]  

#inizializzazione dei thread VideoGet e VideoShow per l'acquisizione dei frame dalla camera
camera = 0

#i parametri KNOWN_DISTANCE, KNOWN_RADIUS sono da aggiornare ogni volta che viene eseguita una nuova calibrazione

resw = 800 # dimensioni (larghezza) in pixel del frame da analizzare
resh = 600 # dimensioni (altezza) in pixel del frame da analizzare

KNOWN_DISTANCE = 2.40 # [m] measured
KNOWN_RADIUS = 0.12 # [m] measured
KNOWN_W = w/f*KNOWN_DISTANCE # Horizontal Field of View - calculated - if possible use measurement
KNOWN_H = h/f*KNOWN_DISTANCE # Vertical Field of View - calculated - if possible use measurement
radius_cal = resw/KNOWN_W*KNOWN_RADIUS # pixel according to resizedframe width - calculated without calibration through immagine_calibrazione.jpg

cap = cv2.VideoCapture(camera)

def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('HSV_thresh_tuning')

h_min = 0
h_max = 6
s_min = 186
s_max = 255
v_min = 0
v_max = 255

# Creating trackbars
cv2.createTrackbar('hMin', 'HSV_thresh_tuning', h_min, 180, nothing)
cv2.createTrackbar('hMax', 'HSV_thresh_tuning', h_max, 180, nothing)
cv2.createTrackbar('sMin', 'HSV_thresh_tuning', s_min, 255, nothing)
cv2.createTrackbar('sMax', 'HSV_thresh_tuning', s_max, 255, nothing)
cv2.createTrackbar('vMin', 'HSV_thresh_tuning', v_min, 255, nothing)
cv2.createTrackbar('vMax', 'HSV_thresh_tuning', v_max, 255, nothing)


def save_contour(cnt):
    with open('calibration/target_contour.txt', 'w') as outfile:
        outfile.write('# Array shape: {0}\n'.format(cnt.shape))

        for data_slice in cnt:
            np.savetxt(outfile, data_slice, fmt='%d')
            outfile.write('# New slice\n')

def load_contour():
    data = np.loadtxt('calibration/target_contour.txt', dtype=np.int32)
    data = data.reshape((data.size / 2, 1, 2))
    return data


if streaming:
    print 'Initializing streaming...'
    up = Upstreamer("upstreaming thread", addr, port, False)
    up.start()

while(1):
    try:
        file = open("target_hu_moments.txt","w")

        #_, frame = cap.read()
        frame = cv2.imread('img/red_ball2.png', cv2.IMREAD_COLOR)   # READ FROM IMG

        #dim = (resw,resh)
        dim = (30*frame.shape[1], 30*frame.shape[0])    # FIXED SCALE
        resizedframe = cv2.resize(frame, dim)

        #converting to HSV
        hsv = cv2.cvtColor(resizedframe,cv2.COLOR_BGR2HSV)

        # get info from track bar and apply to result
        h_min = cv2.getTrackbarPos('hMin','HSV_thresh_tuning')
        h_max = cv2.getTrackbarPos('hMax','HSV_thresh_tuning')
        s_min = cv2.getTrackbarPos('sMin','HSV_thresh_tuning')
        s_max = cv2.getTrackbarPos('sMax','HSV_thresh_tuning')
        v_min = cv2.getTrackbarPos('vMin','HSV_thresh_tuning')
        v_max = cv2.getTrackbarPos('vMax','HSV_thresh_tuning')

        # Normal masking algorithm
        lb1 = np.array([h_min, s_min, v_min])
        ub1 = np.array([h_max, s_max, v_max])
        lb2 = np.array([180-h_max, s_min, v_min])
        ub2 = np.array([180-h_min, s_max, v_max])

        #parametri dell'inquadratura in pixel
        hframe, wframe, _ = resizedframe.shape
        
        # Compute frame center coordinates (pixel)
        cframeX = int(wframe / 2)
        cframeY = int(hframe / 2)
        frame_center = (cframeX, cframeY)
        
        # Draw the middle
        cv2.circle(resizedframe, frame_center, 2, (0, 255, 0), 1)
        
        # create the Mask
        mask1 = cv2.inRange(hsv, lb1, ub1)
        mask2 = cv2.inRange(hsv, lb2, ub2)
        mask = cv2.bitwise_or(mask1, mask2)
        masked = cv2.bitwise_and(resizedframe, resizedframe, mask = mask)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
      
        cv2.drawContours(resizedframe, contours, -1, (255, 0, 0), 2)
        
        circle_hu_moments = [0.160201230483, 0.000288488472415, 3.21161756323e-06, 
                             1.33394542174e-08, 2.04948036639e-15, 1.87087783436e-10, -1.85009253506e-15]

        if len(contours) >= 1:

            # Filter out irrelevant contours (threshold and ranking by area)
            contours = list(filter(lambda cnt: cv2.contourArea(cnt) >= 10, contours))
            list.sort(contours, key = cv2.contourArea, reverse=True)
            contours = contours[:5]

            # Isolate pseudo-circular contours (roundness score 0 -> circle)
            circular_contours = []
            for con in contours:
                roundness = 0
                moments = cv2.moments(con)
                hu_moments = cv2.HuMoments(moments).flatten()
                print(cv2.HuMoments(moments).shape)
                for i in range(0, 7):
                    roundness += (circle_hu_moments[i] - hu_moments[i])**2
                roundness = math.sqrt(roundness)
                if roundness < 0.01:
                    circular_contours.append((con, roundness, hu_moments))
                
            # Sort contours by roundness
            circular_contours.sort(key=lambda c: c[1])
            
            print("=================================\r\n")
            file.write("=================================\r\n")
            file.write("lowerBound=np.array(["+str(h_min)+","+str(s_min)+","+str(v_min)+"])"+"\r\n")
            file.write("upperBound=np.array(["+str(h_max)+","+str(s_max)+","+str(v_max)+"])"+"\r\n")

            if len(circular_contours) > 0:
            
                # The target is the most round object
                target, roundness, hu_moments = circular_contours[0]
                file.write("Best roundness: " + str(roundness) + "\r\n")
                print("Best roundness: " + str(roundness) + "\r\n")
                file.write("Best HU moments: [" + ", ".join(str(x) for x in hu_moments) + "]\r\n")
                print("Best HU moments: [" + ", ".join(str(x) for x in hu_moments) + "]\r\n")
                save_contour(target)

                # Show the target
                x_box, y_box, w_box, h_box = cv2.boundingRect(target)
                cv2.rectangle(resizedframe, (x_box, y_box), (x_box + w_box, y_box + h_box), (0, 0, 255), 2)

                radius = max(w_box, h_box) / 2
                x = x_box + w_box / 2
                y = y_box + h_box / 2

                dist = KNOWN_DISTANCE * radius_cal / radius

                cv2.circle(resizedframe, (int(x), int(y)), int(radius), (0, 255, 255), 1)
                err_x_pix = x - cframeX   # pixel
                err_y_pix = y - cframeY   # pixel

                W = KNOWN_W / KNOWN_DISTANCE * dist # [m]
                H = KNOWN_H / KNOWN_DISTANCE * dist # [m]
                risolW = W / wframe # [m/pixel]
                risolH = H / hframe # [m/pixel]
                err_x_m = risolW * err_x_pix # [m]
                err_y_m = risolH * err_y_pix # [m]
                psi_err = 180 / math.pi * math.atan(err_x_m / dist)   #[deg]
                theta_err = 180 / math.pi * math.atan(err_y_m / dist) #[deg]

                cv2.putText(resizedframe, "Target", (int(x) - 100, int(y) - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(resizedframe, "roundness=%.5f" % (roundness), (int(x) - 100, int(y) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(resizedframe, "dist=%.2fm" % (dist), (0, cframeY + 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(resizedframe, "radius=%.2fm" % (radius*risolH), (0, cframeY + 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(resizedframe, "radius_pix=%.2fm" % (radius), (0, cframeY + 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(resizedframe, "psi=%.2fdeg" % (psi_err), (0, cframeY + 170),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(resizedframe, "theta=%.2fdeg" % (theta_err), (0, cframeY + 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(resizedframe, "err_y=%.2fm" % (err_y_m), (0, cframeY + 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(resizedframe, "err_x=%.2fm" % (err_x_m), (0, cframeY + 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                            
                # Simply visualize the other (non-target) round objects
                for cnt, roundness, hu_moments in circular_contours[1:]:
                    x_box, y_box, w_box, h_box = cv2.boundingRect(cnt)
                    cv2.rectangle(resizedframe, (x_box, y_box), (x_box + w_box, y_box + h_box), (255, 255, 0), 2)
                    cv2.putText(resizedframe, "Other", (int(x_box) - 100, int(y_box) - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                    cv2.putText(resizedframe, "roundness=%.5f" % (roundness), (int(x_box) - 100, int(y_box) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                    file.write("Other roundness: " + str(roundness) + "\r\n")
                    print("Other roundness: " + str(roundness) + "\r\n")
                    file.write("Other HU moments: [" + ", ".join(str(x) for x in hu_moments) + "]\r\n")
                    print("Other HU moments: [" + ", ".join(str(x) for x in hu_moments) + "]\r\n")

        if streaming:
            up.stream_frame(resizedframe)
        else:
            cv2.imshow('calibration', resizedframe)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        break


if streaming:
    up.close()
    
cap.release()

cv2.destroyAllWindows()
