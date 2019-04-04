import threading
import time
import cv2
from queue import Queue, Empty

class VideoGet(threading.Thread):
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def change_res(width, height):
        self.stream.set(3, width)
        self.stream.set(4, height)

    def __init__(self, name, src=0, test=False):
        threading.Thread.__init__(self)
        self.name = name
        self.Q = Queue(1)
        self.__stop_event = threading.Event()

        if test:
            self.stream = cv2.VideoCapture('videos/red_ball_flight.mp4')
        else:
            self.stream = cv2.VideoCapture(src)
        
        #change_res(1920, 1080)
        
        #TODO: Controllo su errori (es. camera errata)
        (ret, frame) = self.stream.read()
        if not ret:
            print 'Cannot get an image. Please make sure the videocamera is plugged'
            raise Exception
        else:
            print 'Opened camera', src

    def stop(self):
        self.__stop_event.set()
        
    def stopped(self):
        return self.__stop_event.is_set()

    def close(self):
        self.stream.release()
    
    def run(self):
        print('Starting new thread for acquisition of frames')
        while not self.stopped():
            if not self.Q.full():
                (ret, frame) = self.stream.read()
                if not ret:
                    print 'Problem with camera: frame acquisition failed.'
                    time.sleep(0.1)
                self.Q.put(frame)
        self.close()
        print 'Stopped thread for acquisition of frames'
        
    def get_frame(self):
        try:
            return self.Q.get(block=True, timeout=2)
        except Empty:   # empty queue
            return None

