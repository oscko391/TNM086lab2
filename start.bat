rem Sets the necessary environment variables for successful execution
rem of SGCT and OSG. Call this script from your Command Window before
rem running SGCT+OSG applications.
rem


set SCRIPT_PATH=%~dp0
call %SCRIPT_PATH%/setenv.bat

start solution.exe -config config/vr_lab_workbench_vive.xml -local 0
start solution.exe -config config/vr_lab_workbench_vive.xml -local 1 --slave
