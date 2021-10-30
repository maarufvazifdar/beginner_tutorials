# Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

### **Author:** *Maaruf Vazifdar*, maarufvazifdar@gmail.com

## Overview
The ROS Pub Sub assignment demonstrates the working of a simple publisher subscriber and nodes communicating over the *chatter* topic.

- **Talker** node continuously publishes a string message on the *chatter* topic at a rate of 10 Hz.
- **Listener** node subscribes to the *chatter* topic and prints the ROS_INFO message on the terminal.

## Dependencies
- ROS - Melodic

## Building and Running the package
1) Build the package and start roscore:
    ```bash
    cd ~/<your_ws>/src
    git clone https://github.com/maarufvazifdar/beginner_tutorials.git
    cd ~/<your_ws>
    catkin_make
    roscore
    ```

2) Run talker node in terminal - 2
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    rosrun beginner_tutorials talker
    ```

3) Run talker node in terminal - 3
    ```bash
    cd ~/<your_ws>
    source devel/setup.bash
    rosrun beginner_tutorials listener
    ```


## Run cpplint and cppcheck
To run Cpplint:
  ```bash
  cpplint $( find . -name *.cpp ) > results/cpplint_result.txt
  ```

To run Cppcheck:
  ```bash
  cppcheck --enable=all --std=c++11 -I src/ --suppress=missingIncludeSystem $( find . -name *.cpp ) > results/cppcheck_result.txt
  ```

## License
MIT License
```
Copyright (c) 2021  Mohammed Maaruf Vazifdar.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```
