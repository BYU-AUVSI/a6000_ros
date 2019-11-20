#!/bin/bash

function gphoto_cleanup() {
    # dont call me unless you're in the gphoto folder
    cd ../
    rm -rf libgphoto2-libgphoto2-2_5_21-release/
    rm libgphoto2-2_5_21-release.tar.gz
}

function installLibGphoto() {

    # figure out what system we're running on
    unameOut="$(uname -s)"
    case "${unameOut}" in
        Linux*)     machine=Linux;;
        Darwin*)    machine=Mac;;
        CYGWIN*)    machine=Cygwin;;
        MINGW*)     machine=MinGw;;
        *)          machine="UNKNOWN:${unameOut}"
    esac

    # Support macOS or linux install
    # dependencies for the dependency. Need these to build libgphoto2 from src
    if [ ${machine} = "Mac" ]; then
        # assume brew exists
        brew install autoconf automake pkg-config gettext libtool libusb libexif
        # since mac already has BSD gettext, force it to link in our brew gettext
        #     so that autoreconf / autopoint work properly
        brew link gettext --force
    elif [ ${machine} = "Linux" ]; then
        sudo apt -y install autoconf autopoint automake pkg-config gettext libtool libusb-1.0-0-dev libexif-dev

        echo "===We need to add you to the plugdev and dialout group, otherwise we cant connect to the camera==="
        sudo adduser $USER dialout
        sudo adduser $USER plugdev
    else
        echo "Sorry, I dont know how to properly install the dev stuff on your type of machine"
        exit 1
    fi

    wget https://github.com/gphoto/libgphoto2/archive/libgphoto2-2_5_21-release.tar.gz

    tar -xf libgphoto2-2_5_21-release.tar.gz

    cd libgphoto2-libgphoto2-2_5_21-release/

    if ! autoreconf --install --symlink; then
        echo "==ERROR: Failed to run autoreconf (likely a gettext install issue?)=="
        gphoto_cleanup
        exit $?
    fi
    if ! ./configure --prefix=/usr/local; then
        echo "==ERROR: Failed to run configure in GPhoto (missing some dependencies?)=="
        gphoto_cleanup
        exit $?
    fi
    if ! make; then
        echo "==ERROR: Failed to build Gphoto=="
        gphoto_cleanup
        exit $?
    fi
    if ! sudo make install; then
        echo "==ERROR: Failed to install Gphoto=="
        gphoto_cleanup
        exit $?
    fi
    
    gphoto_cleanup

    # machine dependent cleanup
    if [ ${machine} = "Mac" ]; then
        # assume brew exists
        # unlink (I dont think we'll need it again since GPhoto is now build and installed)
        brew unlink gettext
    fi
    echo "===DONE!!==="
}

installLibGphoto
