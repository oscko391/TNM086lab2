rem Sets the necessary environment variables for successful execution
rem of SGCT and OSG. Call this script from your Command Window before
rem running SGCT+OSG applications.
rem

set SCRIPT_PATH=%~dp0
set PATH=%PATH%;%SCRIPT_PATH%;C:\VRSystem\OpenSceneGraph-install-vs15\bin
set OSG_LIBRARY_PATH=%OSG_LIBRARY_PATH%;C:\VRSystem\OpenSceneGraph-install-vs15\bin\osgPlugins-3.4.0
set OSG_FILE_PATH=%SCRIPT_PATH%\models
cd /D %SCRIPT_PATH%
