#!/bin/bash
#
# Under Linux, starts two instances (nodes) of the default solution in
# lab2 of TNM086, using the configuration that sets up VRPN tracking,
# and starts the VRPN tracking simulator.
#

SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_PATH

export OSG_FILE_PATH=%SCRIPT_PATH%\models

./solution -config config/linux_lab_VRPN_tracking.xml -local 0         > log_master_output.txt 2> log_master_error.txt &
./solution -config config/linux_lab_VRPN_tracking.xml -local 1 --slave > log_slave_output.txt  2> log_slave_error.txt &

H3DLoad urn:candy:x3d/VRPNServer.x3d &
