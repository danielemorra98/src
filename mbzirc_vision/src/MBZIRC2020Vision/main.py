""" This is the main module used to start the image acquisition,
processing, streaming and navigation control threads. """

import getopt
import socket
import sys
from video_processing.videoprocess import VideoProcessor
from streaming.upstream import Upstreamer
from navcontrol.pilot import Pilot

navcontrol = False  # send navigation commands to the drone?
streaming = False   # stream the video to ground station?
addr = '127.0.0.1'  # default IP address to stream
port = 54321        # default server port to stream
camera = 1          # default camera index
nocolor = False     # colored images (default) or B/W
display = False     # display the captured frame?
test = False        # frames acquired by video instead of camera

try:
    opts, args = getopt.getopt(sys.argv[1:], "hnsdtv:a:p:", \
                 ['help', 'navcontrol', 'streaming', 'nocolor', \
                  'display', 'test', 'videocamera=', 'addr=', 'port='])
except getopt.GetoptError as err:
    print 'Check the README file (or ask Lai) for info about usage.'
    sys.exit(2)
for o, a in opts:
    if o in ("-n", "--navcontrol"):
        navcontrol = True
    elif o in ("-s", "--streaming"):
        streaming = True
    elif o == "--nocolor":
        nocolor = True
    elif o in ("-d", "--display"):
        display = True
    elif o in ("-t", "--test"):
        test = True
    elif o in ("-v", "--videocamera"):
        camera = int(a)
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
        print "Check the README file for info about options."
        sys.exit()
    else:
        assert False, "unhandled option: " + o

# INITIALIZATION
print 'Initializing camera'
vp = VideoProcessor(camera, display, test)
vp.start()
if navcontrol:
    print 'Initializing autopilot'
    pilot = Pilot("autopilot thread")
    pilot.start()
if streaming:
    print 'Initializing streaming'
    up = Upstreamer("upstreaming thread", addr, port, nocolor)
    up.start()

# MAIN LOOP
try:
    while True:
        proc_frame = vp.acquire_processed_image()
        if proc_frame is not None:
            if navcontrol:
                pilot.put_input(proc_frame)
            if streaming:
                up.stream_frame(proc_frame[0])
except KeyboardInterrupt:
    # CLOSING ROUTINE
    print "Starting shutdown routine"
    if navcontrol:
        pilot.close()
    if streaming:
        up.close()
    vp.close()
    print 'All threads stopped'
