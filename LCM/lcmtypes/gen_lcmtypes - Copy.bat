@echo on
copy lcm-gen.exe       lcmdefs
copy libglib-2.0-0.dll lcmdefs
copy libintl-8.dll     lcmdefs
FOR %%x IN (lcmdefs\*.lcm) DO lcmdefs\lcm-gen.exe -x %%x
move irp_slam\*.hpp lcmtypes_cpp
rd /s /q irp_slam
del lcmdefs\lcm-gen.exe
del lcmdefs\libglib-2.0-0.dll
del lcmdefs\libintl-8.dll