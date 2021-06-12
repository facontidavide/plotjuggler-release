
git clone https://github.com/facontidavide/PlotJuggler.git

mkdir build

cd build

cmake ../PlotJuggler/ -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install

make -j8

make install

cd ..

wget https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage

wget https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-x86_64.AppImage

chmod +x linuxdeploy*.AppImage

cd PlotJuggler;export VERSION=$(git describe --abbrev=0 --tags);cd -;echo $VERSION

cp -v install/bin/* AppDir/usr/bin

./linuxdeploy-x86_64.AppImage --appdir=AppDir -d ./PlotJuggler/PlotJuggler.desktop -i ./PlotJuggler/plotjuggler.png --plugin qt --output appimage
