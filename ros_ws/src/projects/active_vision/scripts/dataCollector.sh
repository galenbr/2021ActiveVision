#!/bin/bash

# Script to collect data, save parameters, generate its summary & move to storage folder.

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

cur=$(pwd)
pkgPath=$(rospack find active_vision)
src=$pkgPath"/dataCollected/trainingData/"

csvDataRec='dataRec.csv'
csvParams='parameters.csv'
objectID=(1 2)
nData=(5 5)

# Creating a screen to run ROS & gazebo
printf "Starting gazebo ...\n"
gnome-terminal -- bash -c 'screen -d -R -S session-environment' & sleep 5
# Starting the gazebo and loading the parameters
screen -S session-environment -X stuff $'roslaunch active_vision workspace.launch visual:="OFF"\n'
sleep 10

# Collecting data
for ((i=0;i<${#objectID[@]};++i)); do
		# Setting the rosparams
		rosparam set /active_vision/dataCollectorV2/objID ${objectID[i]}
		rosparam set /active_vision/dataCollectorV2/nData ${nData[i]}
		rosparam set /active_vision/dataCollectorV2/csvName ${csvDataRec}

		# Saving the parameters
		# rosrun active_vision saveParams.sh ${src}${csvParams}

		# Starting a screen and running dataCollectorV2.cpp
		printf "Collecting "${nData[i]}" data points for Object ID : "${objectID[i]}" ..."
		gnome-terminal -- bash -c 'screen -d -R -S session-dataCollection' & sleep 5
		screen -S session-dataCollection -X stuff $'sleep 7\nexit\n' # Dummy line till codes are cleaned
		# screen -S session-dataCollection -X stuff $'rosrun active_vision dataCollectorV2.cpp\nexit\n'

		# Waiting till datacollection is over
		screenOK="$(checkScreen session-dataCollection)"
		while [[ "$screenOK" == "true" ]]; do
			printf ".";	sleep 10
			screenOK="$(checkScreen session-dataCollection)"
		done
		printf "\n"

done

# Closing gazebo
printf "Closing gazebo ..."
screen -S session-environment -X stuff "^C"
screen -S session-environment -X stuff $'sleep 1\nexit\n'

# Waiting till gazebo is closed
screenOK="$(checkScreen session-environment)"
while [[ "$screenOK" == "true" ]]; do
	printf ".";	sleep 5
	screenOK="$(checkScreen session-environment)"
done
printf "\n"

printf "Gazebo closed.\n"; sleep 2

printf "***********\n"

#
# # Get the list of csv files in source directory
# csvList=()
# csvList+=($src$csvDataRec)
# # for file in $(find $src -name "*.csv"); do
# # 	if [ "$(basename $file)" != "$csvParams" ]; then
# #  		csvList+=($file)
# # 	fi
# # done
#
# # Generate Summary and State Vector
# for csv in ${csvList[@]}; do
# 	printf "Using CSV : "$(basename $csv)"\n"
# 	printf "Generating Summary...\n"
# 	rosrun active_vision csvSummarizer.py $csv GRAPH_SAVE
# 	printf "Generating State Vector...\n"
# 	rosrun active_vision genStateVec $(dirname $csv)/ $(basename $csv) 1 5
# done
#
# printf "***********\n"
#
# dst=$pkgPath"/dataCollected/storage/"
#
# # Create a new directory
# i="1"
# ok=false
# while [ $ok = false ]; do
# 	dstToCheck="$dst""Data_$i""/"
# 	if [ ! -d $dstToCheck ]; then
# 		mkdir $dstToCheck
# 		ok=true
# 		printf $(basename $dstToCheck)" folder created.\n"
# 	fi
# 	i=$[$i+1]
# done
#
# # Copying the files to the created folder
# cd $src
# shopt -s extglob
# mv !(ReadMe.txt) $dstToCheck
# shopt -u extglob
# cd $cur
# printf "Files Moved.\n"
#
# printf "***********\n"

# # now="$(date +'%Y_%m_%d_%I_%M_%S')"
# # printf "Current date in dd/mm/yyyy format %s\n" "$now"