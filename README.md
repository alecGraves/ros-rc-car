# rx-arduino-ros
This repository contians necessary instructions and code to interface a standard reciever with an arduino and with ros.

### 1. Compile source:
clone the repository into catkin_ws/src

    cd ~/catkin_ws/src
    git clone --recursive https://github.com/shadySource/rx-arduino-ros/
    catkin_make -j $(nproc) -C "/home/$USER/catkin_ws"

### 2. How to set up Arduino IDE to flash sketch:
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

Install and run arduino:

    sudo apt-get install arduino
    arduino

Then install ros libraries:

    cd ~/sketchbook/libraries
    rosrun rosserial_arduino make_libraries.py .

### 3. Flash the image to the arduino

Connect Arduino

Change permissions on port:

    sudo chmod 777 /dev/ttyACM*

Open IDE:

    arduino

Then open an image image in the IDE and flash it to the arduino.

### 4. Launch serial communication ros node
Run roscore:

    roscore

Run rosserial to recieve data from the arduino:

    rosrun rosserial_python serial_node.py /dev/ttyACM0

