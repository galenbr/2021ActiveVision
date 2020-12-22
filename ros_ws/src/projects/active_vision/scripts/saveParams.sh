#!/bin/bash
#Takes the final .csv file as the argument
dst=$1
prefix="/active_vision/"
parameters=`rosparam list`
filename=`basename $dst`

# If file doesnot exist setup the header
if [[ ! -f "$dst" ]]; then
	printf ${filename}" file setup with headers.\n"
	for param in $parameters; do
		if [ ${param:0:15} == $prefix ]; then
			curParam=${param:15}
			echo -n "$curParam," >> $dst
		fi
	done
	echo "" >> $dst
fi

# Store the parameters to
for param in $parameters; do
	if [ ${param:0:15} == $prefix ]; then
		curValue=`rosparam get $param`
		echo -n "$curValue," >> $dst
	fi
done
printf "Parameters written to "${filename}".\n"
echo "" >> $dst
