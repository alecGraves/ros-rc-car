# rx-arduino-ros
This repository contians necessary instructions and code to interface a standard reciever with an arduino and with ros.

### 1. Compile source:
clone the repository into catkin_ws/src

    cd ~/catkin_ws/src
    git clone --recursive https://github.com/shadySource/rx-arduino-ros/
    catkin_make -j $(nproc) -C "/home/$USER/catkin_ws"

### 2. How to set up Arduino IDE to flash image:
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

The gist of it:

Install and run arduino:

    sudo apt-get install arduino
    arduino

Then install ros libraries:

    cd ~/sketchbook/libraries
    rosrun rosserial_arduino make_libraries.py .

### 3. Flash the image to the arduino

Open IDE:

    sudo arduino
    
Then open the image in the IDE and flash it to the arduino.

### 4. Launch serial communication ros node
Run roscore:

    roscore

Run rosserial to recieve data from the arduino:

    rosrun rosrun rosserial_python serial_node.py /dev/ttyUSB0
    
You may need to adjust the location of the arduino.
