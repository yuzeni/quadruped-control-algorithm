@echo off
setlocal enabledelayedexpansion

set exe_name=Quadruped.exe
set defines=/D ARDUINO_FREE=1
set src_files=..\src\main.cpp
rem /W4 /Wall /O2
set CFlags=/std:c++14 /EHsc /MD /Zi
set include_paths

mkdir build
pushd build
del %exe_name%
cl %CFlags% %defines% %include_paths% %src_files% /Fe:%exe_name%
.\%exe_name%
popd

endlocal
