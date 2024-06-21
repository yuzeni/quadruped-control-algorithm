@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
set path=p:\HotGlue-lang\env\;C:\src\GnuWin32\bin;%path%
P:
cd Quadruped
call emacs.bat
