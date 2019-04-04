import threading
import cv2

class VideoShow(threading.Thread):
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name
        #self.mutex = threading.Lock()
        self.frame = None
        self.__stop_event = threading.Event()

    def stop(self):
        self.__stop_event.set()
        
    def stopped(self):
        return self.__stop_event.is_set()

    def close(self):    
        cv2.destroyAllWindows()

    def run(self):
        print('Starting new thread for showing acquired frames')
        while not self.stopped():
            #self.mutex.acquire()
            if self.frame is not None:
                cv2.imshow("Video", self.frame)
            #self.mutex.release()
            cv2.waitKey(1)
        self.close()
        print 'Stopped thread for showing acquired frames'

    def put_frame(self, new_frame):
        #self.mutex.acquire()
        self.frame = new_frame
        #self.mutex.release()
