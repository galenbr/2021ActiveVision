#!/bin/bash

# Function to check if a screen is still running
checkScreen () {
  local result=false
	shopt -s nullglob
	local screens=(/var/run/screen/S-*/*)
	shopt -u nullglob
	for s in ${screens[*]}; do
			if [[ "$s" == *"$1"* ]]; then
			  result=true
			fi
	done
  echo "$result"
}

# Script to test the provided list of policies on the provided data
cur=$(pwd)
pkgPath=$(rospack find active_vision)
src=$pkgPath"/dataCollected/trainingData/"
dataSrc=$pkgPath"/dataCollected/storage/"

csvDataRec='dataRec.csv'
csvParams='parameters.csv'
csvStorageSummary='storageSummary.csv'
policies=("PCA" "PCA_LDA") #"RANDOM" "HEURISTIC"

files=()

if [[ $# -eq 0 ]]; then
	#Load all files of format Data_N/dataRec_stateVec.csv by default
	echo "All mode"
	dataNo="1"
	ok=true
	while [ $ok = true ]; do
		dstToCheck="$dataSrc""Data_$dataNo""/"
		if [ ! -d $dstToCheck ]; then
			ok=false
		else
			files+=("$dstToCheck""dataRec_stateVec.csv")
		fi
		dataNo=$[$dataNo+1]
	done
else
	#Else load all files in the specified directories
	echo "List mode"
	for item in "$@"; do
		echo "$item"
		dstToCheck="$dataSrc""$item""/"
		files+=("$dstToCheck""dataRec_stateVec.csv")
	done
fi

# Creating a screen to run ROS & gazebo
printf "Starting gazebo ...\n"
gnome-terminal -- bash -c 'screen -d -R -S session-environment' & sleep 5
# Starting the gazebo and loading the parameters
screen -S session-environment -X stuff $'roslaunch active_vision workspace.launch visual:="OFF"\n'
sleep 10

# Looping over the files and testing each policy for each.
for i in ${files[@]}; do
	echo $i
	for policy in ${policies[@]}; do
		# Set each object TODO:Allow user to specify/do all?
		rosparam set /active_vision/dataCollector/objID 1
		rosparam set /active_vision/policyTester/policy $policy

		printf "Evaluating "$policy" on run "$i" ..."

		# Open a terminal for the service and the tester
		gnome-terminal -- bash -c 'screen -d -R -S session-policyService'
		gnome-terminal -- bash -c 'screen -d -R -S session-policyTester'
		sleep 5

		# Start the service and the tester
		screen -S session-policyService -X stuff $'rosrun active_vision trainedPolicyService.py\n'
		#Debug line
		#screen -S session-policyTester -X stuff $'sleep 7\nexit\n'
		screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2\n1\nexit\n'

		# Waiting till testing is over
		screenOK="$(checkScreen session-policyTester)"
		while [[ "$screenOK" == "true" ]]; do
			printf ".";	sleep 60
			screenOK="$(checkScreen session-policyTester)"
		done
		printf "\n"
		screen -S session-policyService -X stuff "^C"
		screen -S session-policyService -X stuff $'sleep 1\nexit\n'
		printf "Starting next policy...\n"
		sleep 2
	done
done
# Closing gazebo
printf "Closing gazebo ...\n"
screen -S session-environment -X stuff "^C"
screen -S session-environment -X stuff $'sleep 1\nexit\n'
