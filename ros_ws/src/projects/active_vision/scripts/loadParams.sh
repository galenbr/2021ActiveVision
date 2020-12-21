#!/bin/bash
#Takes the final .csv file as the argument
destination=$1
parameters=`rosparam list`
first=0
for param in $parameters
do
	if [ ${param:0:15} == "/active_vision/" ]
	then
		curParam=${param:15}
		curValue=`rosparam get $param`
		if [ $first == 0 ]
		then
			echo -n "$curParam,$curValue" >> $destination
			first=1
		else
			echo -n ",$curParam,$curValue" >> $destination
		fi
	fi
done
echo "" >> $destination