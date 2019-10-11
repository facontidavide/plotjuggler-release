# First time

Add the right repository form [here](https://launchpad.net/~beineri)


    sudo add-apt-repository ppa:beineri/opt-qt597-xenial -y

Then, run:

    sudo apt-get update
    sudo apt-get install qt59base qt59svg qt59declarative qt59translations -y

Download the latest version of [LinuxDeployQt](https://github.com/probonopd/linuxdeployqt) and make it executable with __chmod__:

    wget "https://github.com/probonopd/linuxdeployqt/releases/download/continuous/linuxdeployqt-continuous-x86_64.AppImage" -O ~/linuxdeployq.AppImage
    chmod +x ~/linuxdeployq.AppImage
    
    sudo apt-get -y install libgtk2.0-dev
    git clone http://code.qt.io/qt/qtstyleplugins.git
    cd qtstyleplugins
    source /opt/qt59/bin/qt59-env.sh
    /opt/qt59/bin/qmake
    make -j$(nproc)
    sudo make install 
    cd ..
    rm -rf qtstyleplugins

# Build the AppImage with catkin_make

In the root folder of ws_plotjuggler:

    rm -rf build devel install
    source /opt/qt59/bin/qt59-env.sh
    catkin_make -DCMAKE_BUILD_TYPE=Release  -j$(nproc) install  
    unset QTDIR; unset QT_PLUGIN_PATH ; unset LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$PWD/install/lib
    
    
    
    cd src/PlotJuggler;export VERSION=$(git describe --abbrev=0 --tags);cd -;echo $VERSION
    
    ~/linuxdeployq.AppImage ./install/lib/plotjuggler/PlotJuggler.desktop -appimage -bundle-non-qt-libs -no-strip \ 
         -extra-plugins=iconengines,imageformats,platformthemes/libqgtk2.so,styles/libqgtk2style.so
          



