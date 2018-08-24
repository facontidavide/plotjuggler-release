Ubuntu 14.04 build (Semaphore): [![Build Status](https://semaphoreci.com/api/v1/facontidavide/plotjuggler/branches/master/shields_badge.svg)](https://semaphoreci.com/facontidavide/plotjuggler)

ROS Indigo/Kinetic build (Travis): [![Build Status](https://travis-ci.org/facontidavide/PlotJuggler.svg?branch=master)](https://travis-ci.org/facontidavide/PlotJuggler)

[![Join the chat at https://gitter.im/PlotJuggler/Lobby](https://badges.gitter.im/PlotJuggler/Lobby.svg)](https://gitter.im/PlotJuggler/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# PlotJuggler 1.8.3

QT5 based application to display time series in plots. 

To understand what PlotJuggler can do for you, take a look to the following video [PlotJuggler on Vimeo](https://vimeo.com/214389001) 

![PlotJuggler](docs/images/PlotJuggler.gif)

# How to build (non ROS users)

Clone the repository as usual:

      git clone https://github.com/facontidavide/PlotJuggler.git

The only binary dependency that you need installed in your system is Qt5. On Ubuntu the debians can be installed with the command:

    sudo apt-get -y install qtbase5-dev libqt5svg5-dev
    
On Fedora:

    sudo dnf install qt5-qtbase-devel qt5-qtsvg-devel
    
Then compile using cmake (qmake is NOT supported):

     mkdir build; cd build
     cmake ..
     make
     sudo make install
 
 Note: the plugins need to be installed in the same folder of the executable or in __/usr/local/lib/PlotJuggler/__.

# How to build (ROS users)

 The following instructions are for __ROS Kinetic__. Adapt them accordingly if you are using a different version of ROS.
 
 The easiest way to install PlotJuggler is through the command:
 
    sudo apt-get install ros-kinetic-plotjuggler 

Nevertheless, if you want to compile it from source, for instance to try the very latest version on the master branch, you __must__ build PlotJuggler using __catkin__, otherwise the ROS related plugins will not be included.

Follow these instructions:

    sudo apt-get install qtbase5-dev libqt5svg5-dev ros-kinetic-ros-type-introspection 
    mkdir -p ws_plotjuggler/src; cd ws_plotjuggler/src
    git clone https://github.com/facontidavide/PlotJuggler.git
    cd ..
    catkin_make
    source devel/setup.bash
    
You should see the following message at the beginning of the compilation step:

    "PlotJuggler is being built using CATKIN. ROS plugins will be compiled"

Both the executable and the plugins will be created in __ws_plotjuggler/devel/lib/plotjuggler__.

To run the application, use the command:

    rosrun plotjuggler PlotJuggler 

# How you may help

PlotJuggler required many hundreds of man/hours to be developed, with the goal to be the most
intuitive tool to visualize data and time series.

If you believe that I achieved this goal, consider making a donation here: [Paypal](https://www.paypal.me/facontidavide).

If you have any problem, you found a bug or you need help, feel free to submit an issue.

      

