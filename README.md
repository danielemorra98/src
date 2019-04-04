# MBZIRC SSSUP PACKAGE

## WARNING: INSTRUCTION FOR UBUNTU 18.04 AND ROS MELODIC
If you have Ubuntu 16.04 it may still work but you have to change commands accordingly

##  1) Install Required Components
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update
sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros python-wstool python-catkin-tools 
sudo apt-get install protobuf-compiler libgoogle-glog-dev
sudo apt-get install geographiclib-tools 
sudo apt-get install libgeographic-dev
sudo apt-get install ros-melodic-opencv-apps
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install git gitk git-gui 
sudo apt-get install build-essential cmake git libboost-all-dev mercurial libcegui-mk2-dev libopenal-dev libswscale-dev libavformat-dev libavcodec-dev  libltdl3-dev libqwt-dev ruby libusb-1.0-0-dev libbullet-dev libhdf5-dev libgraphviz-dev libgdal-dev
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
-------------------------------------------------
## 2) Install Ardupilot + SITL:

##### 2.1):Clone Ardupilot Repository
```
cd ~
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
```

##### 2.2) Install Prerequisites
```
Tools/scripts/install-prereqs-ubuntu.sh -y 
. ~/.profile  
 ```
##### 2.2) ALTERNATIVE- ONLY if you didn’t run the install-prereqs script from previous step. 

Add the following lines to the end of your “.bashrc” in your home directory
(notice the . on the start of that filename. Also, this is a hidden file,
 so if you’re using a file manager, make sure to turn on “show hidden files”).
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

```
Then reload your PATH by using the “dot” command in a terminal, oppure anche solo chiudete il terminale e riapritelo
```
. ~/.bashrc
```
Reference:
http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
http://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux


## 3) Verify if Ardupilot+Sitl is working standalone
```
cd ardupilot/ArduCopter
sim_vehicle.py -j4 --map --console
```
And in the terminal
```
mode GUIDED
arm throttle
takeoff 100 
```
Eventually right click in the Mavlink map and use the fly to function

-------------------------
# we missed a step ;) Download gazebo

```curl -sSL http://get.gazebosim.org | sh```
Reference:http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install

Note that, if you go to gazebosim.org, you may also download the .deb pacage, or compile gazebo from source. THe one above is the easiest way.

# Test gazebo
in a terminal, ```gazebo --verbose```

# ATTENTION: POSSIBLE ERROR!
If gazebo doesn't start, starts with a black screen or throws some errors like those ones, it may be caused by your graphical card settings/problem/performance/drivers. For istance, it happened on virtual machines, and on old crappy machines.

-----------------------
# 4) Download gazebo model database 

Reference:
https://web.archive.org/web/20180503141535/http://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html

```
cd ~
mkdir ~/gazebo_ws
cd ~/gazebo_ws/
hg clone https://bitbucket.org/osrf/gazebo_models ~/gazebo_ws/gazebo_models
cd ~/gazebo_ws/gazebo_models
hg checkout zephyr_demos
echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/gazebo_ws/gazebo_models' >> ~/.bashrc
source ~/.bashrc
```

#####  4.1) Verify is Ardupilot+Sitl is working WITH GAZEBO

# 4.1.1)
In one terminal, enter the ArduCopter directory and start the SITL simulation:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -f gazebo-iris -D --console --map
```
#####  4.1.2) In another terminal start Gazebo, with the world including the iris quadcopter:
```
cd ~/gazebo
gazebo --verbose worlds/iris_arducopter_demo.world
```
WARNING: This command shold only work if you downloaded gazebo models
Plus, you may need to export the path to the gazebo model database in your bashrc;
Otherwise, you could(theoretically, but sitll it isn't tested) skip step 4.0, and start gazebo alone.
It should prompt you that it's downloading the model database from the internet, and that it may (and will) take some time. 
After that, gazebo is probably gonna save the downloaded models into ~/.gazebo/gazebo_models, or something similar

## 5) Modify ARDUCOPTER Directory

You must do two operations in arducopter: modify the parameters for the custom drone and add the custom frame.
Aggiungere i parametri mav relativi al drone 
 
In ```ardupilot/Tools/autotest```
 
##### 5.1)Copy the folder MBZIRC-provaParams
You find this folder in ~/catkin_ws/src/mbzirc_SITL
 
In ```ardupilot/Tools/autotest/pysim```
#####  5.2)Modify '''vehicleinfo.py''' :
Right under:
 ```
"gazebo-iris": {
   "waf_target": "bin/arducopter",
   "default_params_filename": ["default_params/copter.parm",
                               "default_params/gazebo-iris.parm"],
},
```
you should paste:
```
#progetto roccella
"gazebo-MBZIRColo":{
    "waf_target": "bin/arducopter",
    "default_params_filename":["default_params/copter.parm",
                                "MBZIRC-provaParams/gazebo-MBZIRColo.parm"],
},
```
From  ```ardupilot/ArduCopter``` try to launch ```sim_vehicle.py --console -f gazebo-MBZIRColo``` 
After compiling, it should be waiting for an heartbeat (from Gazebo) on the port 127.0.0.1:... 

------------------------------------------------------------
## 6) Create the new catkin_ws and clone required packages
```
mkdir -p catkin_ws/src

```
-----------------------
# cd catkin_ws/src 
# catkin_init_workspace
these instruction needs verification!
i suggest 

```cd catkikin_ws
catkin_init_workspace 
```
or, better/you can do both
```catkin init --workspace .
```

---------------------

```cd ..
rm -r src
git clone https://github.com/Bochicchio3/src.git
cd ..
catkin build
source devel/setup.bash
```

```
echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/$(rospack find mbzirc_SITL)/gazebo_models' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/$(catkin locate)/build/mbzirc_SITL' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins/' >> ~/.bashrc
```

##### USEFUL HINTS:

IF YOU ARE RUNNING ON VIRTUAL MACHINE RUN
``` 
echo `export SVGA_VGPU10=0`>> ~/.bashrc
```
IF YOU DON'T WANT TO SOURCE YOUR WORKSPACE EVERY TIME
```
echo 'source $(catkin locate)/devel/setup.bash'>> ~/.bashrc
```
--------------------------------------------------------------------------------
