# MBZIRC20_Code
Code of the drone for the MBZIRC 2020 challenge 

## Files and Folders

- **main.py**: the main script, usually what you need to execute
- **calibration**: scripts to calibrate the detection algorithms
- **docs**: useful documentation (check it out)
- **img**: various images useful for testing or training
- **navcontrol**: python files for drone guidance and navigation  
- **streaming**: python files for video streaming
- **video_processing**: python files for image acquisition and object detection
- **videos**: various videos for testing or training
- **README**: what you're reading right now  

## Execute code

The most versatile way to launch the code (except downstream) is executing `main.py` script, specifying some options.

`python main.py <options>`

**Options**

You may add the following options at the end:
* `-n` or `--navcontrol`: Enable sending navigation commands to the drone
* `-s` or `--streaming`: Enable video streaming
* `-a <ip>` or `--addr <ip>`: Specify the IP address of the streamer (i.e. the drone). Default is 127.0.0.1.
* `-p <number>` or `--port <number>`: Specify the port of the video stream receiver. Default is 54321.
* `-v <index>` or `--videocamera <index>`: Specify the index of videocamera for video input acquisition. Default is 1.
* `-d` or `--display`: Show the acquired images on the screen.
* `-t` or `--test`: Acquire frames from video instead of videocamera
* `-h` or `--help`: Show info about the usage

## Streaming ##

If you want to see the streaming, first make sure that the main program is running on the drone (with the appropriate options), then launch the downstream script on your pc:

`python downstream.py`

**Options**

You may add the following options to downstream:
* `-a <ip>` or `--addr <ip>`: Specify the IP address of the streamer. Default is 127.0.0.1.
* `-p <number>` or `--port <number>`: Specify the port where you want to receive the data stream. Default is 54321.
* `-h` or `--help`: Show info about the usage
