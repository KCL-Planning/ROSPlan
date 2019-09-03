#!/bin/bash
# Author: Gerard Canal <gcanal@iri.upc.edu>
# This script runs the rddlsim and prost together, to work as a single planner executable
# Usage: ./run_prost_online.sh RDDL_DOMAIN_FILE RDDL_PROBLEM_FILE SEARCH_OPTIONS <PLANNER_OUTPUT_FILE> (Both rddl files must be in the same folder!)

########################################################################################################################
ARGC=$#
if [[ $ARGC -lt 3 ]]; then
	echo -e "Usage: ./run_prost_online.sh RDDL_DOMAIN_FILE RDDL_PROBLEM_FILE SEARCH_OPTIONS <PLANNER_OUTPUT_FILE>\n(Both rddl files must be in the same folder!)"
	exit 1
fi

## Get arguments
RDDL_DOMAIN_FILE=$(realpath $1)
RDDL_PROBLEM_FILE=$(realpath $2)
SEARCH_OPTIONS=$3
PLANNER_OUTPUT_FILE=$4
WAIT_SERVER_TIME=0.5  # Seconds

if [[ -z "$PLANNER_OUTPUT_FILE" ]]; then
	PLANNER_OUTPUT_FILE=/dev/null
fi;

########################################################################################################################
## Check file existances
if [ ! -f "$RDDL_DOMAIN_FILE" ]; then
  echo "Error: Provided RDDL domain file $RDDL_DOMAIN_FILE was not found!"
  exit 2
fi

if [ ! -f "$RDDL_PROBLEM_FILE" ]; then
  echo "Error: Provided RDDL instance file $RDDL_PROBLEM_FILE was not found!"
  exit 2
fi

PROST_HOME="$(rospack find rosplan_planning_system)/common/bin/prost"
if [ ! -d "$PROST_HOME" ]; then
  echo "Error: prost was not found in $PROST_HOME"
  exit 2
fi

########################################################################################################################
## Get the files folders and join them for rddlsim
FILES_FOLDER_PATH=$(dirname $RDDL_DOMAIN_FILE)
PROBLEM_FOLDER_PATH=$(dirname $RDDL_PROBLEM_FILE)

MOVED_PROBLEM_FILE=0;
if [[ "$FILES_FOLDER_PATH" != "$PROBLEM_FOLDER_PATH" ]]; then
	cp $RDDL_PROBLEM_FILE $FILES_FOLDER_PATH;
	MOVED_PROBLEM_FILE=1;
	RDDL_PROBLEM_FILE=$FILES_FOLDER_PATH/$(basename $RDDL_PROBLEM_FILE)
fi

########################################################################################################################
# Get instance name from PROBLEM FILE
INSTANCE_NAME=$(sed -n 's/instance \(.*\) {/\1/p' $RDDL_PROBLEM_FILE)

########################################################################################################################
########################################################################################################################
########################################################################################################################
## Run the parser and planner
SERVER_PORT=$(rosparam get /rosplan_plan_dispatcher/ippc_server_port 2>/dev/null) # Get it from the parameter
if [[ -z "$SERVER_PORT" ]]; then  # Set default
	SERVER_PORT=3234
fi

# Wait for the server to be online
# The above command gets the PID of a process using the port $SERVER_PORT
SERVER_PID=$(netstat -tulpn 2>&1 | grep $SERVER_PORT | sed -n 's_.*\s\([0-9]\+\)/.*_\1_p')
while [[ -z $SERVER_PID ]]; do
	SERVER_PID=$(netstat -tulpn 2>&1 | grep $SERVER_PORT | sed -n 's_.*\s\([0-9]\+\)/.*_\1_p') 
	sleep $WAIT_SERVER_TIME # Give time for the server to die
done

########################################################################################################################
########################################################################################################################
########################################################################################################################
cd $PROST_HOME
rm $INSTANCE_NAME parser_*.rddl parser_out*  >/dev/null 2>&1 # Clean-up previous files, in case some were left

# Run prost planner (To change to any other IPPC planner only this section needs to be changed)
export LIBC_FATAL_STDERR_=1 # To avoid printing planner errors: https://stackoverflow.com/a/4616162
$PROST_HOME/prost $INSTANCE_NAME -h localhost -p $SERVER_PORT $SEARCH_OPTIONS >$PLANNER_OUTPUT_FILE 2>&1
EXIT_CODE=$?
########################################################################################################################
########################################################################################################################
########################################################################################################################

### Cleanup
rm $INSTANCE_NAME parser_*.rddl parser_out* >/dev/null 2>&1 # Clean-up generated files
if [[ $MOVED_PROBLEM_FILE -eq 1 ]]; then
	rm $RDDL_PROBLEM_FILE;
fi

exit $EXIT_CODE
########################################################################################################################
########################################################################################################################
########################################################################################################################

# Debug variables
echo RDDL_DOMAIN_FILE $RDDL_DOMAIN_FILE
echo RDDL_PROBLEM_FILE $RDDL_PROBLEM_FILE
echo SEARCH_OPTIONS $SEARCH_OPTIONS
echo FILES_FOLDER_PATH $FILES_FOLDER_PATH
echo PROBLEM_FOLDER_PATH $PROBLEM_FOLDER_PATH
echo MOVED_PROBLEM_FILE $MOVED_PROBLEM_FILE
echo INSTANCE_NAME $INSTANCE_NAME
echo RDDLSIM_PID $RDDLSIM_PID
exit
