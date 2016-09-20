@echo on
copy ..\lcm-1.2.1\WinSpecific\Release\lcm-gen.exe       lcmdefs
copy ..\lcm-1.2.1\WinSpecific\glibs\lib\libglib-2.0-0.dll lcmdefs
copy ..\lcm-1.2.1\WinSpecific\glibs\lib\libintl-8.dll     lcmdefs
copy ..\lcm-1.2.1\WinSpecific\glibs\lib\libiconv-2.dll     lcmdefs
FOR %%x IN (lcmdefs\*.lcm) DO lcmdefs\lcm-gen.exe -c %%x
FOR %%x IN (lcmdefs\*.lcm) DO lcmdefs\lcm-gen.exe -x %%x
FOR %%x IN (lcmdefs\*.lcm) DO lcmdefs\lcm-gen.exe -j %%x
FOR %%x IN (lcmdefs\*.lcm) DO lcmdefs\lcm-gen.exe -p %%x
move *.c lcmtypes_c
move *.h lcmtypes_c
move irp_slam\*.hpp lcmtypes_cpp
move irp_slam\*.java lcmtypes_jar
move irp_slam\*.py lcmtypes_py
rd /s /q irp_slam
del lcmdefs\lcm-gen.exe
del lcmdefs\libglib-2.0-0.dll
del lcmdefs\libintl-8.dll
del lcmdefs\libiconv-2.dll