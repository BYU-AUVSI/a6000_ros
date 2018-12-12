# dependencies for the dependency. Need these to build libgphoto2 from src
sudo apt -y install autoconf autopoint automake pkg-config gettext libtool libusb-1.0-0-dev libexif-dev

wget https://github.com/gphoto/libgphoto2/archive/libgphoto2-2_5_21-release.tar.gz

tar -xvf libgphoto2-2_5_21-release.tar.gz

cd libgphoto2-libgphoto2-2_5_21-release/

autoreconf --install --symlink
./configure --prefix=/usr/local
make
sudo make install

# need newer version of cmake than what comes with 16.04 from aptitude
#NOTE this only works for x86 processors - need something different for ARM
arch="$(uname -m)"

if [ ${arch} = "x86_64" ]; then

    sudo apt -y remove cmake
    cd /opt/
    sudo wget https://github.com/Kitware/CMake/releases/download/v3.13.1/cmake-3.13.1-Linux-x86_64.sh
    sudo chmod +x cmake-3.13.1-Linux-x86_64.sh 

    #running this askes for a bunch of yeses
    sudo ./cmake-3.13.1-Linux-x86_64.sh 

    sudo ln -s /opt/cmake-3.13.1-Linux-x86_64/bin/* /usr/local/bin
else
    echo "Hey there. Tyler knows how to install cmake from source on non-x86 platforms, but he's lazy and hasn't put it in this script yet..."
fi