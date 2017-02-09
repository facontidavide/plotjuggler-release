[![Build Status](https://travis-ci.org/facontidavide/PlotJuggler.svg?branch=master)](https://travis-ci.org/facontidavide/PlotJuggler)

[![Join the chat at https://gitter.im/PlotJuggler/Lobby](https://badges.gitter.im/PlotJuggler/Lobby.svg)](https://gitter.im/PlotJuggler/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# PlotJuggler

QT5 based application to display time series in plots. This is under heavy development. 

To understand what PlotJuggler can do for you, take a look to the following video.

<iframe src="https://player.vimeo.com/video/174120477" width="640" height="360" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe> <p><a href="https://vimeo.com/174120477">PlotJuggler: a desktop application to plot time series.</a></p>

![PlotJuggler](/PlotJuggler.gif)

# How to build

First of all you need to clone the repository and its submodules either using the command:

      git clone https://github.com/facontidavide/PlotJuggler.git

The only binary dependencies that you need installed on your system is Qt5. On Ubuntu the debians can be installed with the command:

    sudo apt-get -y install qtbase5-dev 
    
On Fedora

    sudo dnf install qt5-qtbase-devel
    
Then proceed as you would do with any cmake based project

     mkdir build; cd build
     cmake ..
     make
     sudo make install
 
 Note: you should not skip the last installation step. Currently the plugins need to be installed in the folder __/usr/local/lib/PlotJuggler/__ otherwise PlotJuggle will not find them.
 
# Note for ROS users
 
If you use CATKIN to build this project, the ROS related plugins will be automatically included into the compilation.
If you __don't want__ to compile the ROS plugins but catkin is installed in your system,  comment this line in CMakeLists.txt

       find_package(catkin QUIET)

You will also need to download and build this package: https://github.com/facontidavide/ros_type_introspection 

__IMPORTANT__: there is a critical bug in version 0.3.x of ros_type_introspection. Use version 0.4.X or later.

__At the time of writing, you must compile it from source (master branch).__

In the future you will be able to install it on ROS Indigo and Kinetic:

       sudo apt-get install ros-indigo-ros-type-introspection
or

       sudo apt-get install ros-kinetic-ros-type-introspection
      

