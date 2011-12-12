#!/bin/sh
# Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
#   and the University of Michigan
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# cv2cg.sh driver script file

####################
# Function
####################
# Waits for user key press.
pause()
{
 OLDCONFIG=`stty -g`
 stty -icanon -echo min 1 time 0
 dd count=1 2>/dev/null
 stty $OLDCONFIG
}

####################
# Main
####################
clear

echo [cv2cg] camera file = $1
echo [cv2cg] image1 file = $2
echo [cv2cg] image2 file = $3

#spare reconstruction
echo [cv2cg] 1.SparseRec2View
SparseRec2View $1 $2 $3
pause

#simulate
echo [cv2cg] 2.CameraSimulator.exe
CameraSimulator $2-$3.X
echo $2>>$2-$3.X.out.swt
echo $2.par>>$2-$3.X.out.swt
echo $3>>$2-$3.X.out.swt
echo $3.par>>$2-$3.X.out.swt
pause

echo [cv2cg] 3.CameraSwitcher.exe
CameraSwitcher $2-$3.X.out.swt
