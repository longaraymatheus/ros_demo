# ros_demo

ros_demo is a package that provides usage examples of [ROS](http://www.ros.org) (Robot Operating Systems).

* Publisher/subscriber
* Service/client
* Camera publisher/subscriber using [OpenCV](http://opencv.org/)

Licensed under MIT license. Check [LICENSE](/LICENSE) for more info.

## Installing

* Clone this repo on your catkin\_workspace: `git clone https://github.com/longaraymatheus/ros_demo.git`
* Make: `catkin_make install`
* See below how to use provided node examples

## Usage examples

Make sure you have [roscore](http://wiki.ros.org/roscore) up and running.

For publisher/subscriber:
* In one tab, run: `rosrun basic_demo basic_demo_publisher`
* In another one, run: `rosrun basic_demo basic_demo_listener`

For service/client:
* In one tab, run: `rosrun basic_demo basic_demo_service`
* In another one, run: `rosrun basic_demo basic_demo_client X Y`
  * Pass whatever number you like for X and Y

For camera publisher/subscriber:
* In one tab, run: `rosrun opencv_demo opencv_demo_publisher DEVICE_ID`
  * Make sure you pass correct camera device ID
* In another one, run: `rosrun opencv_demo opencv_demo_listener`

## Contributing

Please, feel free to create a pull request and submit changes and/or other usage examples :)
