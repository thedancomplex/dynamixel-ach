#!/bin/bash

UTILS_NAME=dynamixel-ach-utils
PROJECT_NAME=dynamixel-ach
INSTALL_DIR=/etc/$PROJECT_NAME
SERVER_NAME=server_$PROJECT_NAME
DIR_SERVER=server/server
DIR_SCRIPTS=scripts



sudo mkdir -p $INSTALL_DIR
sudo cp -r * $INSTALL_DIR/

# Install Server
cd $INSTALL_DIR/$DIR_SERVER
sudo ./build.sh

# Make Startup ScriptSimlinks
cd $INSTALL_DIR/$DIR_SCRIPTS

sudo rm /usr/bin/$PROJECT_NAME
sudo rm /usr/bin/$UTILS_NAME
sudo ln -s $INSTALL_DIR/$DIR_SCRIPTS/$PROJECT_NAME /usr/bin
sudo ln -s $INSTALL_DIR/$DIR_SCRIPTS/$UTILS_NAME /usr/bin
 
