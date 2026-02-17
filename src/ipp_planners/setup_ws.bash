#!/bin/bash
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

WS_DIR="$SCRIPT_DIR"/../..

# link the .vscode folder to the workspace root ${WS_DIR}
while true; do
	read -p "This will link the ipp_planners/ws_files/.vscode folder to the workspace root. Are you sure? (y/n): " yn
    case $yn in
        [Yy]* ) echo "Linking .vscode to workspace root..."; ln -sr `readlink "$SCRIPT_DIR"/ws_files/.vscode` "$WS_DIR"; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

while true; do
	read -p "This will import the git repos from ipp_planners/ws.repos. Are you sure? (y/n): " yn
    case $yn in
        [Yy]* ) echo "Importing ROS packages from ipp_planners/ws.repos to $WS_DIR/src ..."; cd "$WS_DIR"/src && vcs import < ipp_planners/ws.repos && echo "Done."; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo
echo "This VSCode setup is configured to use catkin-tools. Make sure you use 'catkin build' commands, not catkin_make."
echo "Also be sure to install the catkin-tools VSCode extension: https://marketplace.visualstudio.com/items?itemName=betwo.b2-catkin-tools"

echo "Workspace setup done."
