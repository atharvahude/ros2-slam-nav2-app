#!/bin/bash
echo "Removing build, log, and install folders. Building and sourcing install/setup.bash"
rm -rf build log install
colcon build
source ./install/setup.bash
