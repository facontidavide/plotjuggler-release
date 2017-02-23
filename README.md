[![Build Status](https://travis-ci.org/facontidavide/PlotJuggler.svg?branch=master)](https://travis-ci.org/facontidavide/PlotJuggler)

[![Join the chat at https://gitter.im/PlotJuggler/Lobby](https://badges.gitter.im/PlotJuggler/Lobby.svg)](https://gitter.im/PlotJuggler/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# PlotJuggler

QT5 based application to display time series in plots. This is under heavy development. 

To understand what PlotJuggler can do for you, take a look to the following video.

<iframe src="https://player.vimeo.com/video/174120477" width="640" height="360" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe> <p><a href="https://vimeo.com/174120477">PlotJuggler: a desktop application to plot time series.</a></p>

![PlotJuggler](/PlotJuggler.gif)

# How to build

Clone the repository as usual:

      git clone https://github.com/facontidavide/PlotJuggler.git

The only binary dependency that you need installed in your system is Qt5. On Ubuntu the debians can be installed with the command:

    sudo apt-get -y install qtbase5-dev 
    
On Fedora

    sudo dnf install qt5-qtbase-devel
    
Then compile using cmake (qmake is NOT supported).

     mkdir build; cd build
     cmake ..
     make
     sudo make install
 
 Note: the plugins need to be installed in the same folder of the executable or in __/usr/local/lib/PlotJuggler/__.
 
# Note for ROS users
 
If you use CATKIN to build this project, the ROS related plugins will be automatically included into the compilation. Both the executable and the plugins will be created in __devel/lib/plotjuggler__ (the address relative to the catkin workspace).

To run the application, use the command:

    rosrun plotjuggler PlotJuggler
    
Alternatively, just execute the binary __PlotJuggler__. 

A mandatory dependency is the package: https://github.com/facontidavide/ros_type_introspection 

__IMPORTANT__: there is a critical bug in version 0.3.x of ros_type_introspection. Use version 0.4.3 or later.

__At the time of writing, you must compile ros_type_introspection from source (master branch).__

In the future you will be able to install it on ROS Indigo and Kinetic:

       sudo apt-get install ros-indigo-ros-type-introspection
or

       sudo apt-get install ros-kinetic-ros-type-introspection
      

