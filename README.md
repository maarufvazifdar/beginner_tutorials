# Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

### **Author:** *Maaruf Vazifdar*, maarufvazifdar@gmail.com

## Overview
The ROS assignment demonstrates the working of a simple publisher subscriber and nodes communicating over the *chatter* topic and a starts a service to change the base_string in talker.

- **Talker** node continuously publishes a string message on the *chatter* topic at a rate (in Hz) according to the argument given when launching.
- **Listener** node subscribes to the *chatter* topic and prints the ROS_INFO message on the terminal.

## Dependencies
- ROS - Melodic

## Building and Running
1) Build the package and launch talker and listener nodes:
    ```bash
    cd ~/<your_ws>/src
    git clone https://github.com/maarufvazifdar/beginner_tutorials.git
    cd ~/<your_ws>
    catkin_make
    roslaunch beginner_tutorials beginner_tutorials.launch talker_frequency:=20
    ```

2) Run call rosservice *my_service* in terminal - 2
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    rosservice call /my_service "input: 'new_string'" 
    ```

3) Run rqt_console in terminal - 3
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    rosrun rqt_console rqt_console 
    ```

4) Run rqt_logger_level in terminal - 4
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    rosrun rqt_logger_level rqt_logger_level 
    ```

    Screenshot of rqt_console and rqt_logger_levels
    ![](/results/ros_logger_image.png)

## Run cpplint and cppcheck
To run Cpplint:
  ```bash
  cpplint $( find . -name *.cpp ) > results/cpplint_result.txt
  ```

To run Cppcheck:
  ```bash
  cppcheck --language=c++ --std=c++11 -I include --suppress=missingIncludeSystem  $( find . -name \*.hpp -or -name \*.cpp) > results/cppcheck_result.txt
  ```

## License
MIT License
```
Copyright (c) 2021  Mohammed Maaruf Vazifdar.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```
