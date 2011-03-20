:: Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)
::   and the University of Michigan
::
:: This program is free software; you can redistribute it and/or modify
:: it under the terms of the GNU General Public License as published by
:: the Free Software Foundation; either version 2 of the License, or
:: (at your option) any later version.
::
:: This program is distributed in the hope that it will be useful,
:: but WITHOUT ANY WARRANTY; without even the implied warranty of
:: MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
:: GNU General Public License for more details.
::
:: cv2cg.bat driver script file

@echo off
cls
set exe_2view=%~dp0%SparseRec2View.exe
set exe_simul=%~dp0%CameraSimulator.exe
set exe_switc=%~dp0%CameraSwitcher.exe

echo [cv2cg] camera file = %1
echo [cv2cg] image1 file = %2
echo [cv2cg] image2 file = %3

::spare reconstruction
echo [cv2cg] 1.SparseRec2View
%exe_2view% %1 %2 %3
pause

::simulate
echo [cv2cg] 2.CameraSimulator.exe
%exe_simul% %~dpn2-%~n3.X
echo %2>>%~dpn2-%~n3.X.out.swt
echo %2.par>>%~dpn2-%~n3.X.out.swt
echo %3>>%~dpn2-%~n3.X.out.swt
echo %3.par>>%~dpn2-%~n3.X.out.swt
pause

echo [cv2cg] 3.CameraSwitcher.exe
%exe_switc% %~dpn2-%~n3.X.out.swt
