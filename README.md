How to run the arm controller:

1. plug in the correct odrives to motors, via USB hub to PC, and then power the + and - rails with <= 40V (20V recommended for testing)
    DO NOT TEST WITH LIPO!!! 
    the motors will draw a high current and they can very easily be overdrawn, killing the batteries

2. make sure python3 is installed
    python3 --version

3. install dependencies
    pip3 install odrive
    pip3 install adafruit-circuitpython-bno055
    pip3 install multiprocessing
    pip3 install PyQt5

4. run program
    cd src
    python3 arm_main.py
