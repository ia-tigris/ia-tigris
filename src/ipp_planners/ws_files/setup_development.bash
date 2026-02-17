#!/bin/bash
ln -sr $(readlink -f ./.vscode) ../../../ || echo ".vscode already exists in ws root, so NOT symlinking this one. Please check if you want to delete your original one and rerun this command."

echo "Setting development workspace to build in DEBUG mode"
set -x
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-O0
