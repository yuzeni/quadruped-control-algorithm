@echo off
setlocal enabledelayedexpansion

set exe_name=Quadruped.exe
set defines=/D ARDUINO_FREE=1 /D VISUALIZE_QUADRUPED=1
set src_files= ..\src\main.cpp ..\src\visualize_quadruped.cpp
rem /W4 /Wall /O2
set CFlags=/std:c++14 /EHsc /MD /Zi 
set include_paths /I..\raylib50

mkdir build
pushd build
del %exe_name%
cl %CFlags% %defines% %include_paths% %src_files% /Fe:%exe_name% gdi32.lib msvcrt.lib ..\raylib50\raylib.lib user32.lib shell32.lib winmm.lib /link /libpath:..\raylib50
.\%exe_name%
popd

endlocal
