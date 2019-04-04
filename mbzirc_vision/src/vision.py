#!/usr/bin/env python



# -*- coding: cp1252 -*-
""" This module performs image-processing, including object detection """

from std_msgs.msg import String
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Point
import math
import os
import numpy as np
from videoget import VideoGet
from videoshow import VideoShow
from queue import Queue, Empty
from timeit import default_timer as timer
from functools import wraps
import cv_algorithms as cv
import state_estimation as se

# Color filter thresholds (hue wraps around 0 for better detection of red)
H_MIN = 0       # Hue
H_MAX = 10
S_MIN = 135     # Saturation
S_MAX = 255
V_MIN = 0       # Value
V_MAX = 255

# Hu moments
CIRCLE_HU_MOMENTS = (0.159964692588, 0.000213728293322, 1.07243178846e-06, 4.48481871921e-09,
                     2.99043207098e-16, 5.44148782025e-11, -8.55147708365e-17)

def load_contour(filename):
    """ Retrieve the target contour stored in a txt file. """
    data = np.loadtxt(filename, dtype=np.int32)
    data = data.reshape((data.size / 2, 1, 2))
    return data

lower_bound1 = np.array([H_MIN, S_MIN, V_MIN])
upper_bound1 = np.array([H_MAX, S_MAX, V_MAX])
lower_bound2 = np.array([180 - H_MAX, S_MIN, V_MIN]) # symmetrical wrt H=0
upper_bound2 = np.array([180 - H_MIN, S_MAX, V_MAX])
target_contour = load_contour('/home/apm/catkin_ws/src/vision/src/calibration/target_contour.txt')

def print_object_info(self, frame, x, y, roundness, highlight=False):
    """ Print object information on top of the given frame """

    cv2.putText(frame, "Target" if highlight else "Other", (int(x) - 100, int(y) - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    cv2.putText(frame, "roundness=%.3f" % (roundness), (int(x) - 100, int(y) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    return frame


def print_target_global_info(self, frame, dist, radius, radius_pix,
                                psi_err, theta_err, err_y_m, err_x_m, frame_center_y):
    """ Print metrics of the object on top of the given frame """

    cv2.putText(frame, "dist=%.2fm" % (dist), (0, frame_center_y + 200),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, "radius=%.2fm" % (radius), (0, frame_center_y + 170),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(frame, "radius_pix=%.2f" % (radius), (0, frame_center_y + 140),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(frame, "psi=%.2fdeg" % (psi_err), (0, frame_center_y + 110),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(frame, "theta=%.2fdeg" % (theta_err), (0, frame_center_y + 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(frame, "err_y=%.2fm" % (err_y_m), (0, frame_center_y + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(frame, "err_x=%.2fm" % (err_x_m), (0, frame_center_y + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    return frame


class Follower:



    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('/gimbal/image_raw', 
                                        Image, self.image_callback)
        self.pixeloutput=rospy.Publisher('/positionforservo',Point,queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                        Twist, queue_size=1)
        self.twist = Twist()
        self.w_frame = 0
        self.h_frame = 0
        self.frame_center_x = 0
        self.frame_center_y = 0
        self.x = 0
        self.y = 0
        self.radius_pix = 0
        self.pixel_coordinate=Point()

    def get_distance(self):

        return CALIBR_DIST * CALIBR_BALL_RADIUS_PIX / self.radius_pix


    def _get_pos_error_pix(self):

        err_x_pix = self.x - self.frame_center_x  	# [pixel] x offset from the center
        err_y_pix = self.y - self.frame_center_y  	# [pixel] y offset from the center

        return err_x_pix, err_y_pix


    def get_pos_error_m(self):

        err_x_pix, err_y_pix = self._get_pos_error_pix()
        dist = self.get_distance()

        fov_w = CALIBR_FOV_W / CALIBR_DIST * dist   # [m] horizontal field of view at balloon distance
        fov_h = CALIBR_FOV_H / CALIBR_DIST * dist   # [m] vertical field of view at balloon distance
        resol_w = fov_w / self.w_frame              # [m/pixel] horizontal resolution 
        resol_h = fov_h / self.h_frame              # [m/pixel] vertical resolution
        err_x_m = resol_w * err_x_pix               # [m] x error from the center
        err_y_m = resol_h * err_y_pix               # [m] y error from the center

        return err_x_m, err_y_m


    def get_pos_error_angle(self):

        dist = self.get_distance()
        err_x_m, err_y_m = self.get_pos_error_m()

        psi_err = 180 / math.pi * math.atan(err_x_m / dist)   #[deg] yaw error
        theta_err = 180 / math.pi * math.atan(err_y_m / dist) #[deg] pitch error

        return psi_err, theta_err


    def get_recomputed_radius(self):

        dist = self.get_distance()

        fov_w = CALIBR_FOV_W / CALIBR_DIST * dist   # [m] horizontal field of view at balloon distance
        fov_h = CALIBR_FOV_H / CALIBR_DIST * dist   # [m] vertical field of view at balloon distance
        resol_w = fov_w / self.w_frame              # [m/pixel] horizontal resolution
        resol_h = fov_h / self.h_frame              # [m/pixel] vertical resolution

        radius = self.radius_pix * resol_w          # [m] object radius (computed again for validation)

        return radius


    def put_observation(self, x, y, radius_pix, w_frame, h_frame):

        self.w_frame = w_frame
        self.h_frame = h_frame
        self.frame_center_x = int(self.w_frame / 2)
        self.frame_center_y = int(self.h_frame / 2)

        self.x = x
        self.y = y
        self.radius_pix = radius_pix

    def image_callback(self, msg):
        # Convert to hsv
        frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        frame_hsv = cv.bgr_to_hsv(frame)
        # Isolate red regions
        red_mask = cv.color_filter(frame_hsv, lower_bound1, upper_bound1, lower_bound2, upper_bound2)
        # Detect contours
        contours = cv.find_contours(red_mask)
        
        if len(contours) >= 1:
            # Compute the area for each contour
            contours = map(lambda cnt: (cnt, cv2.contourArea(cnt)), contours)
            # Filter out irrelevant contours (threshold and ranking by area)
            contours = list(filter(lambda cnt: cnt[1] >= 3, contours))
            list.sort(contours, key = lambda cnt: cnt[1], reverse=True)
            contours = contours[:5]

            # Isolate pseudo-circular contours (roundness score 0 -> circle)
            circular_contours = []
            for con, area in contours:
                '''
                roundness1 = cv2.matchShapes(con, self.target_contour, 1, 0.0)
                roundness2 = cv2.matchShapes(con, self.target_contour, 2, 0.0)
                roundness3 = cv2.matchShapes(con, self.target_contour, 3, 0.0)
                print('Area: %d   R1: %f   R2: %f   R3: %f' % (area, roundness1, roundness2, roundness3))    #DEBUG
                '''
                # Compare its Hu moments with those of the balloon
                roundness = cv2.matchShapes(con,target_contour, 3, 0.0)
                if roundness < 0.35:
                    circular_contours.append((con, roundness))   
                

            # Sort contours by roundness
            circular_contours.sort(key=lambda c: c[1])

            if len(circular_contours) > 0:
                # Chose as target the roundest object
                target, roundness = circular_contours[0]
                
                # Show it
                x_box, y_box, w_box, h_box = cv2.boundingRect(target)
                cv2.rectangle(frame, (x_box, y_box), (x_box + w_box, y_box + h_box), (0, 0, 255), 2)

                # Approximate position of the balloon in the frame
                radius_pix = max(w_box, h_box) / 2  # [pixel] radius
                x = x_box + w_box / 2               # [pixel] center x
                y = y_box + h_box / 2               # [pixel] center y

                # Compute the approximated state
                # self.state_estimator.put_observation(x, y, radius_pix, w_frame, h_frame)
                # dist = self.state_estimator.get_distance()
                # err_x_pix, err_y_pix = self.state_estimator._get_pos_error_pix()
                # err_x_m, err_y_m = self.state_estimator.get_pos_error_m()
                # psi_err, theta_err = self.state_estimator.get_pos_error_angle()
                # radius = self.state_estimator.get_recomputed_radius()

                # Draw a circle around the target
                cv2.circle(frame, (int(x), int(y)), int(radius_pix), (0, 255, 255), 1)
                cv2.imshow("output", frame)
                cv2.waitKey(3)
                # Print info about the target
                frame = print_object_info(frame, x, y, roundness, True)
                # frame = print_target_global_info(frame, dist, radius, radius_pix,
                #                                       psi_err, theta_err, err_y_m, err_x_m, frame_center_y)

                # Simply visualize the other (non-target) round objects
                for cnt, roundness in circular_contours[1:]:
                    x_box, y_box, w_box, h_box = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x_box, y_box), (x_box + w_box, y_box + h_box), (255, 255, 0), 2)
                    frame = print_object_info(frame, x_box, y_box, roundness, False)
               
                self.pixel_coordinate.x=x
                self.pixel_coordinate.y=y
                self.pixel_coordinate.z=0
                self.pixeloutput.publish(self.pixel_coordinate)
                

rospy.init_node('vision_algorithm')
follower = Follower()
rospy.spin()




       









# def timing(f):
#     """ Decorator to measure the execution time of a function. """
#     @wraps(f)
#     def wrapper(*args, **kwargs):
#         start = timer()
#         result = f(*args, **kwargs)
#         end = timer()
#         print('%s: execution time: %.4f' %(f.__name__, (end - start)))
#         return result
#     return wrapper
#
#
# def load_contour(filename):
#     """ Retrieve the target contour stored in a txt file. """
#     data = np.loadtxt(filename, dtype=np.int32)
#     data = data.reshape((data.size / 2, 1, 2))
#     return data
#
#
# class VideoProcessor(threading.Thread):
#     """ Continuously processes frames from Videoget with a dedicated thread.
#     Attributes
#     ----------
#     video_getter : threading.Thread
#         image acquisition thread
#     video_show : threading.Thread
#         image display thread
#     Methods
#     -------
#     stop()
#         Signal the image processor to terminate as soon as possible.
#     stopped()
#         Returns true if signaled to stop.
#     close()
#         Perform closing operations (shutdown threads).
#     init_color_thresh()
#         Initialize color filtering thresholds.
#     process_image()
#         Extract info from a frame (object detection, position estimation)
#     acquire_processed_image()
#         Acquire a new fully processed frame.
#     """
#
#     def __init__(self, camera_id=1, show=False, test=False):
#         """
#         Parameters
#         ----------
#         camera_id : int, optional
#             The index of the videocamera
#         show : bool, optional
#             Display the acquired image
#         test : bool, optional
#             Acquire the image from a recorded video instead of videocamera
#         """
#
#         threading.Thread.__init__(self)
#         self.__stop_event = threading.Event()
#
#         # Buffer of processed images (size 1)
#         self.Q = Queue(1)
#         self.show = show
#
#         # Start threads VideoGet and VideoShow
#         self.video_getter = VideoGet("Acquisition thread", camera_id, test)
#         self.video_getter.start()
#         if self.show:
#             self.video_shower = VideoShow("Show thread")
#             self.video_shower.start()
#
#         # State estimator (balloon position, error, ...)
#         self.state_estimator = se.StateEstimator()
#
#         self.init_color_thresh()
#         #self.init_morph_kernels()
#         self.init_hu_moments()
#         self.target_contour = load_contour('calibration/target_contour.txt')
#
#
#     def stop(self):
#         """ Signal the image processor to terminate as soon as possible. """
#
#         self.__stop_event.set()
#
#
#     def stopped(self):
#         """ Returns true if signaled to stop. """
#
#         return self.__stop_event.is_set()
#
#
#     def close(self):
#         """ Perform closing operations (shutdown threads). """
#
#         if self.show:
#             self.video_shower.stop()
#             self.video_shower.join()
#         self.video_getter.stop()
#         self.video_getter.join()
#         self.stop()
#         self.join()
#
#
#     def init_color_thresh(self):
#         """ Initialize color filtering thresholds. """
#
#         self.lower_bound1 = np.array([H_MIN, S_MIN, V_MIN])
#         self.upper_bound1 = np.array([H_MAX, S_MAX, V_MAX])
#         self.lower_bound2 = np.array([180 - H_MAX, S_MIN, V_MIN]) # symmetrical wrt H=0
#         self.upper_bound2 = np.array([180 - H_MIN, S_MAX, V_MAX])
#
#     '''
#     def init_morph_kernels(self):
#         """ Initialize kernels for morphological transformations. """
#         self.kernel_open = np.ones((5, 5))
#         self.kernel_close = np.ones((50, 50))
#     '''
#
#
#     def init_hu_moments(self):
#         """ Initialize Hu moments. """
#
#         self.circle_hu_moments = CIRCLE_HU_MOMENTS
#
#
#     def scale_frame(self, frame, w=800):
#         """ Scale the frame to match the specified width """
#
#         ratio = w / (frame.shape[1] * 1.0)
#         dim = (w, int(frame.shape[0] * ratio))
#         resizedframe = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
#         hframe, wframe, _ = resizedframe.shape
#         return resizedframe, hframe, wframe
#
#

#

#
#     @timing
#     def process_image(self, frame):
#         """ Manipulate the given image to identify the target (balloons) """
#
#         # Resize
#         frame, h_frame, w_frame = self.scale_frame(frame, 800)
#
#         # Compute frame center coordinates (pixel)
#         frame_center_x = int(w_frame / 2)
#         frame_center_y = int(h_frame / 2)
#
#         # Convert to hsv
#         frame_hsv = cv.bgr_to_hsv(frame)
#
#         # Isolate red regions
#         red_mask = cv.color_filter(frame_hsv, self.lower_bound1, self.upper_bound1, self.lower_bound2, self.upper_bound2)
#
#         # masked = cv2.bitwise_and(frame, frame, mask=red_mask) # DEBUG
#         # return pf.ProcessedFrame(masked, 0, None, None, None, None, None, None, None) #DEBUG
#
#         # Detect contours
#         contours = cv.find_contours(red_mask)
#
#         if len(contours) >= 1:
#             # Compute the area for each contour
#             contours = map(lambda cnt: (cnt, cv2.contourArea(cnt)), contours)
#             # Filter out irrelevant contours (threshold and ranking by area)
#             contours = list(filter(lambda cnt: cnt[1] >= 3, contours))
#             list.sort(contours, key = lambda cnt: cnt[1], reverse=True)
#             contours = contours[:5]
#
#             # Isolate pseudo-circular contours (roundness score 0 -> circle)
#             circular_contours = []
#             for con, area in contours:
#                 '''
#                 roundness1 = cv2.matchShapes(con, self.target_contour, 1, 0.0)
#                 roundness2 = cv2.matchShapes(con, self.target_contour, 2, 0.0)
#                 roundness3 = cv2.matchShapes(con, self.target_contour, 3, 0.0)
#                 print('Area: %d   R1: %f   R2: %f   R3: %f' % (area, roundness1, roundness2, roundness3))    #DEBUG
#                 '''
#                 # Compare its Hu moments with those of the balloon
#                 roundness = cv2.matchShapes(con, self.target_contour, 3, 0.0)
#                 if roundness < 0.35:
#                     circular_contours.append((con, roundness))
#
#
#             # Sort contours by roundness
#             circular_contours.sort(key=lambda c: c[1])
#
#             if len(circular_contours) > 0:
#                 # Chose as target the roundest object
#                 target, roundness = circular_contours[0]
#
#                 # Show it
#                 x_box, y_box, w_box, h_box = cv2.boundingRect(target)
#                 cv2.rectangle(frame, (x_box, y_box), (x_box + w_box, y_box + h_box), (0, 0, 255), 2)
#
#                 # Approximate position of the balloon in the frame
#                 radius_pix = max(w_box, h_box) / 2  # [pixel] radius
#                 x = x_box + w_box / 2               # [pixel] center x
#                 y = y_box + h_box / 2               # [pixel] center y
#
#                 # Compute the approximated state
#                 self.state_estimator.put_observation(x, y, radius_pix, w_frame, h_frame)
#                 dist = self.state_estimator.get_distance()
#                 err_x_pix, err_y_pix = self.state_estimator._get_pos_error_pix()
#                 err_x_m, err_y_m = self.state_estimator.get_pos_error_m()
#                 psi_err, theta_err = self.state_estimator.get_pos_error_angle()
#                 radius = self.state_estimator.get_recomputed_radius()
#
#                 # Draw a circle around the target
#                 cv2.circle(frame, (int(x), int(y)), int(radius_pix), (0, 255, 255), 1)
#
#                 # Print info about the target
#                 frame = self.print_object_info(frame, x, y, roundness, True)
#                 frame = self.print_target_global_info(frame, dist, radius, radius_pix,
#                                                       psi_err, theta_err, err_y_m, err_x_m, frame_center_y)
#
#                 # Simply visualize the other (non-target) round objects
#                 for cnt, roundness in circular_contours[1:]:
#                     x_box, y_box, w_box, h_box = cv2.boundingRect(cnt)
#                     cv2.rectangle(frame, (x_box, y_box), (x_box + w_box, y_box + h_box), (255, 255, 0), 2)
#                     frame = self.print_object_info(frame, x_box, y_box, roundness, False)
#
#                 return pf.ProcessedFrame(frame, 1, err_x_pix, err_y_pix, err_x_m, err_y_m,
#                                          dist, psi_err, theta_err)
#
#         return pf.ProcessedFrame(frame, 0, None, None, None, None, None, None, None)
#
#
#     def acquire_processed_image(self):
#         """ Acquire a new fully processed frame.
#         Try to get a processed image before the request times out. If timeout, None is returned.
#         """
#         try:
#             return self.Q.get(True, 0.5)
#         except Empty:
#             print 'Image acquisition timeout (0.5 s)'
#             return None
#
#
# if __name__ == "__main__":
#
#     print 'Initializing camera'
#     vp = VideoProcessor(1, True)
#     vp.start()
#     try:
#         while(True):
#             vp.acquire_processed_image()
#     except KeyboardInterrupt:
#         vp.close()
# print 'All threads stopped'
