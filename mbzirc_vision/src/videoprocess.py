# -*- coding: cp1252 -*-
""" This module performs image-processing, including object detection """

import math
import os
import threading
import cv2
import numpy as np
from videoget import VideoGet
from videoshow import VideoShow
from queue import Queue, Empty
from timeit import default_timer as timer
from functools import wraps
from enum import Enum
import processedframe as pf
import cv_algorithms as cv
import state_estimation as se


# Color filter thresholds (hue wraps around 0 for better detection of red)
H_MIN = 0       # Hue
H_MAX = 10
S_MIN = 135     # Saturation
S_MAX = 255
V_MIN = 80      # Value
V_MAX = 255

# Hu moments
CIRCLE_HU_MOMENTS = (0.159964692588, 0.000213728293322, 1.07243178846e-06, 4.48481871921e-09, 
                     2.99043207098e-16, 5.44148782025e-11, -8.55147708365e-17)

# Detection/tracking balance
MAX_CONSECUTIVE_TRACKS = 20


def timing(f):
    """ Decorator to measure the execution time of a function. """
    @wraps(f)
    def wrapper(*args, **kwargs):
        start = timer()
        result = f(*args, **kwargs)
        end = timer()
        print('%s: execution time: %.4f' %(f.__name__, (end - start)))
        return result
    return wrapper


def load_contour(filename):
    """ Retrieve the target contour stored in a txt file. """
    data = np.loadtxt(filename, dtype=np.int32)
    data = data.reshape((data.size / 2, 1, 2))
    return data


class VPMode(Enum):
    """ Video Processor modes """
    detection = 1   # object detection
    tracking = 2    # object tracking


class VideoProcessor(threading.Thread):
    """ Continuously processes frames from Videoget with a dedicated thread. 

    Attributes
    ----------
    video_getter : threading.Thread
        image acquisition thread
    video_show : threading.Thread
        image display thread

    Methods
    -------
    stop()
        Signal the image processor to terminate as soon as possible.
    stopped()
        Returns true if signaled to stop.
    close()
        Perform closing operations (shutdown threads).
    init_color_thresh()
        Initialize color filtering thresholds.
    process_image()
        Extract info from a frame (object detection, position estimation)
    acquire_processed_image()
        Acquire a new fully processed frame.
    """

    def __init__(self, camera_id=1, show=False, test=False):
        """
        Parameters
        ----------
        camera_id : int, optional
            The index of the videocamera
        show : bool, optional
            Display the acquired image 
        test : bool, optional
            Acquire the image from a recorded video instead of videocamera
        """

        threading.Thread.__init__(self)
        self.__stop_event = threading.Event()

        # Buffer of processed images (size 1)
        self.Q = Queue(1)
        self.show = show

        # Start threads VideoGet and VideoShow
        self.video_getter = VideoGet("Acquisition thread", camera_id, test)
        self.video_getter.start()
        if self.show:
            self.video_shower = VideoShow("Show thread")
            self.video_shower.start()

        # State estimator (balloon position, error, ...)
        self.state_estimator = se.StateEstimator()

        self.init_color_thresh()
        #self.init_morph_kernels()
        self.init_hu_moments()
        self.target_contour = load_contour('calibration/target_contour.txt')

        # Tracker and detector configuration
        self.tracker = None
        self.tracked_cnt = 0        # number of consecutive frames tracked
        self.mode = VPMode.detection


    def stop(self):
        """ Signal the image processor to terminate as soon as possible. """

        self.__stop_event.set()


    def stopped(self):
        """ Returns true if signaled to stop. """

        return self.__stop_event.is_set()


    def close(self):
        """ Perform closing operations (shutdown threads). """

        if self.show:
            self.video_shower.stop()
            self.video_shower.join()
        self.video_getter.stop()
        self.video_getter.join()
        self.stop()
        self.join()


    def init_color_thresh(self):
        """ Initialize color filtering thresholds. """

        self.lower_bound1 = np.array([H_MIN, S_MIN, V_MIN])
        self.upper_bound1 = np.array([H_MAX, S_MAX, V_MAX])
        self.lower_bound2 = np.array([180 - H_MAX, S_MIN, V_MIN]) # symmetrical wrt H=0
        self.upper_bound2 = np.array([180 - H_MIN, S_MAX, V_MAX])

    '''
    def init_morph_kernels(self):
        """ Initialize kernels for morphological transformations. """

        self.kernel_open = np.ones((5, 5))
        self.kernel_close = np.ones((50, 50))
    '''


    def init_hu_moments(self):
        """ Initialize Hu moments. """

        self.circle_hu_moments = CIRCLE_HU_MOMENTS


    def scale_frame(self, frame, w=800):
        """ Scale the frame to match the specified width """

        ratio = w / (frame.shape[1] * 1.0)
        dim = (w, int(frame.shape[0] * ratio))
        resizedframe = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        hframe, wframe, _ = resizedframe.shape
        return resizedframe, hframe, wframe


    def print_object_info(self, frame, x, y, highlight=False):
        """ Print object information on top of the given frame """

        cv2.putText(frame, "Target" if highlight else "Other", (int(x) - 100, int(y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        return frame


    def print_target_global_info(self, frame, dist, radius, radius_pix,
                                 psi_err, theta_err, err_y_m, err_x_m, frame_center_y):
        """ Print metrics of the object on top of the given frame """

        cv2.putText(frame, "dist=%.2fm" % (dist), (0, frame_center_y + 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "radius=%.2fm" % (radius), (0, frame_center_y + 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(frame, "radius_pix=%.2f" % (radius_pix), (0, frame_center_y + 140),
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


    def run(self):
        """ Acquisition routine. """

        print 'Starting new thread for processing acquired frames'
        while not self.stopped():
            if not self.Q.full():
                # Get frame
                frame = self.video_getter.get_frame()
                if frame is not None:
                    # Process it
                    proc_frame = self.process_image(frame)
                    # Store
                    self.Q.put(proc_frame)
                    # Show (optional)
                    if self.show:
                        self.video_shower.put_frame(proc_frame.frame)
        print 'Stopped thread for processing acquired frames'


    #@timing
    def process_image(self, frame):
        """ Manipulate the given image to identify the target (balloons) """

        # Resize
        frame, h_frame, w_frame = self.scale_frame(frame, 800)
        
        # Compute frame center coordinates (pixel)
        frame_center_x = int(w_frame / 2)
        frame_center_y = int(h_frame / 2)

        # Copy of the frame, to be edited and used later as output
        frame_edited = frame.copy()
        cv2.putText(frame_edited, "Detecting" if self.mode == VPMode.detection else "Tracking" , 
                (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Target position in the image (initialized to not found)
        targ_x_box, targ_y_box, targ_w_box, targ_h_box = (None,)*4

        if self.mode == VPMode.detection:
            # Convert to hsv
            frame_hsv = cv.bgr_to_hsv(frame)
            
            # Isolate red regions
            red_mask = cv.color_filter(frame_hsv, self.lower_bound1, self.upper_bound1, self.lower_bound2, self.upper_bound2)
            
            # masked = cv2.bitwise_and(frame, frame, mask=red_mask) # DEBUG
            #return pf.ProcessedFrame(masked, 0, None, None, None, None, None, None, None) #DEBUG

            # Detect contours
            contours = cv.find_contours(red_mask)
            circular_contours = []

            if len(contours) >= 1:
                # Compute the area for each contour
                contours = map(lambda cnt: (cnt, cv2.contourArea(cnt)), contours)
                # Filter out irrelevant contours (threshold and ranking by area)
                contours = list(filter(lambda cnt: cnt[1] >= 3, contours))
                list.sort(contours, key = lambda cnt: cnt[1], reverse=True)
                contours = contours[:5]

                # Isolate pseudo-circular contours (roundness score 0 -> circle)
                for con, area in contours:
                    # Compare its Hu moments with those of the balloon
                    roundness = cv2.matchShapes(con, self.target_contour, 3, 0.0)
                    if roundness < 0.35:
                        circular_contours.append((con, roundness))
                    
                # Sort contours by roundness
                circular_contours.sort(key=lambda c: c[1])

            # Secondary (non-target) round objects: simply visualize them
            if len(circular_contours) > 1: 
                for cnt, roundness in circular_contours[1:]:
                    x_box, y_box, w_box, h_box = cv2.boundingRect(cnt)
                    cv2.rectangle(frame_edited, (x_box, y_box), (x_box + w_box, y_box + h_box), (255, 255, 0), 2)
                    frame_edited = self.print_object_info(frame_edited, x_box, y_box, False)

            # Main target: visualize and compute state
            if len(circular_contours) > 0:
                # Chose as target the roundest object
                target, roundness = circular_contours[0]
                targ_x_box, targ_y_box, targ_w_box, targ_h_box = cv2.boundingRect(target)
                # (Re)initialize the tracker
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(frame, (targ_x_box, targ_y_box, targ_w_box, targ_h_box))
                self.mode = VPMode.tracking
                self.tracked_cnt = 0
        else:
            (success, box) = self.tracker.update(frame)
            if success:
                (targ_x_box, targ_y_box, targ_w_box, targ_h_box) = [int(v) for v in box]
                self.tracked_cnt = (self.tracked_cnt + 1) % MAX_CONSECUTIVE_TRACKS
            if not success or self.tracked_cnt == 0:
                self.mode = VPMode.detection

        if targ_x_box is not None:  # if the target was detected
            # Show the target
            # cv2.rectangle(frame_edited, (targ_x_box, targ_y_box), (targ_x_box + targ_w_box, targ_y_box + targ_h_box), (0, 0, 255), 2)

            # Approximate position of the balloon in the frame
            radius_pix = max(targ_w_box, targ_h_box) / 2  # [pixel] radius
            x = targ_x_box + targ_w_box / 2               # [pixel] center x
            y = targ_y_box + targ_h_box / 2               # [pixel] center y

            # Compute the approximated state
            self.state_estimator.put_observation(x, y, radius_pix, w_frame, h_frame)
            dist = self.state_estimator.get_distance()
            err_x_pix, err_y_pix = self.state_estimator._get_pos_error_pix()
            err_x_m, err_y_m = self.state_estimator.get_pos_error_m()
            psi_err, theta_err = self.state_estimator.get_pos_error_angle()
            radius = self.state_estimator.get_recomputed_radius()

            # Draw a circle around the target
            cv2.circle(frame_edited, (int(x), int(y)), int(radius_pix), (0, 255, 255), 1)

            # Print info about the target
            frame_edited = self.print_object_info(frame_edited, x, y, True)
            frame_edited = self.print_target_global_info(frame_edited, dist, radius, radius_pix,
                                                  psi_err, theta_err, err_y_m, err_x_m, frame_center_y)

            return pf.ProcessedFrame(frame_edited, 1, err_x_pix, err_y_pix, err_x_m, err_y_m,
                                     dist, psi_err, theta_err)

        return pf.ProcessedFrame(frame_edited, 0, None, None, None, None, None, None, None)


    def acquire_processed_image(self):
        """ Acquire a new fully processed frame.

        Try to get a processed image before the request times out. If timeout, None is returned.
        """
        try:
            return self.Q.get(True, 0.5)
        except Empty:
            print 'Image acquisition timeout (0.5 s)'
            return None


if __name__ == "__main__":

    print 'Initializing camera'
    vp = VideoProcessor(1, True)
    vp.start()
    try:
        while(True):
            vp.acquire_processed_image()
    except KeyboardInterrupt:
        vp.close()
        print 'All threads stopped'
