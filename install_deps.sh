#!/bin/bash

function installLibGphoto() {
    # dependencies for the dependency. Need these to build libgphoto2 from src
    sudo apt -y install autoconf autopoint automake pkg-config gettext libtool libusb-1.0-0-dev libexif-dev

    echo "===We need to add you to the plugdev and dialout group, otherwise we cant connect to the camera==="
    sudo adduser $USER dialout
    sudo adduser $USER plugdev

    wget https://github.com/gphoto/libgphoto2/archive/libgphoto2-2_5_21-release.tar.gz

    tar -xf libgphoto2-2_5_21-release.tar.gz

    cd libgphoto2-libgphoto2-2_5_21-release/

    autoreconf --install --symlink
    ./configure --prefix=/usr/local
    make
    sudo make install
    
    cd ../
    rm -rf libgphoto2-libgphoto2-2_5_21-release/
    rm libgphoto2-2_5_21-release.tar.gz
    echo "===DONE!!==="
}

installLibGphoto
