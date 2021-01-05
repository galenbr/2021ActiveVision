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

csvDataRec='dataRec.csv'
csvParams='parameters.csv'
csvStorageSummary='storageSummary.csv'
policies=("RANDOM" "PCA" "PCA_LDA" "HEURISTIC")

files=()

#Load every .csv file in the user suplied directory
dataSrc=$pkgPath"$1"
for file in $(ls $dataSrc | grep .csv); do
	echo "Loaded file $file ... "
	files+=("$file")
done

# Creating a screen to run ROS & gazebo
printf "Starting gazebo ...\n"
gnome-terminal -- bash -c 'screen -d -R -S session-environment' & sleep 5
# Starting the gazebo and loading the parameters
screen -S session-environment -X stuff $'roslaunch active_vision workspace.launch visual:="OFF"\n'
sleep 10

#If the user supplied an object number, set it, otherwise use the default.
if [[ $# > 1 ]]; then
	echo "Using object id $2"
	rosparam set /active_vision/policyTester/objID $2
fi

# Looping over the files and testing each policy for each.
for i in ${files[@]}; do
	echo $i
	for policy in ${policies[@]}; do
		rosparam set /active_vision/policyTester/policy $policy
		#All files are found in the user input directory
		rosparam set /active_vision/policyTester/storageDir $1
		rosparam set /active_vision/policyTester/csvStVec $i
		#Save the results to policyfile_dataRec.csv (triming the extra .csv)
		rosparam set /active_vision/policyTester/csvName "${policy}_${i: 0: -4}_dataRec.csv"


		printf "Evaluating $policy on run ${i: 0: -4} ..."

		# Open a terminal for the service and the tester
		gnome-terminal -- bash -c 'screen -d -R -S session-policyService'
		gnome-terminal -- bash -c 'screen -d -R -S session-policyTester'
		sleep 5

		# Start the service and the tester
		if [[ "$policy" == "HEURISTIC" ]]; then
			screen -S session-policyService -X stuff $'rosrun active_vision heuristicPolicyService 0\n'
			screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 1\n1\n exit\n'
		else
			screen -S session-policyService -X stuff $'rosrun active_vision trainedPolicyService.py\n'
			screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2\n1\n exit\n'
		fi
		sleep 5
		screen -S session-policyTester -X stuff $'sleep 1\nexit\n'
		#Debug line
		#screen -S session-policyTester -X stuff $'sleep 7\nexit\n'
		

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
