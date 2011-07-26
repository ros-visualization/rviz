#!/bin/sh
#
# Shell script to print information about the computer, for use in reporting rviz bugs.

echo "Graphics card"
lspci | grep VGA
echo

echo "OS"
cat /etc/issue

echo "Machine"
uname -a
echo

if [ -x /usr/bin/glxinfo ]; then
    echo "OpenGL driver version"
    glxinfo | grep OpenGL
    echo
else
    echo "ERROR: glxinfo command not found.  To find opengl driver version info, please install it with this command:"
    echo "  sudo apt-get install mesa-utils"
    echo "and re-run $0"
fi
