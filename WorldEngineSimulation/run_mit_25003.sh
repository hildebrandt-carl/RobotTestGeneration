#!/bin/zsh

# Get the variables passed to the bash script
savedirectory=$1 
searchtype=$2
scoretype=$3
saveprefix=$4
trajlength=$5
searchtime=$6
controllertype=$7

if [ -z "$savedirectory" ] || [ -z "$searchtype" ]  || [ -z "$scoretype" ]  || [ -z "$saveprefix" ] || [ -z "$trajlength" ] 
then
      echo "Please run this script using ./script.sh <your_directory> <search_type> <score_type> <save_prefix> <trajectory_length>"
	  exit 1
fi

# Change the ROS_MASTER_URI to allow multiple version of ROS to run at the same time
ROS_MASTER_URI=http://localhost:11313

# Source the ros workspace
source devel/setup.zsh

# # Used to save the port number
port=25003

if [ -z "$searchtime" ]
then
	totaltime=3600
else
	totaltime=($searchtime)
fi

# Create a temporary unity folder #TODO
mkdir -p ./tmp_dir/
cp -r ../Unity/Build ./tmp_dir/
mv ./tmp_dir/Build ./tmp_dir/Build${port}

# Go into the directory
cd ./tmp_dir/Build${port}

# Get current directory
current_dir="$PWD"

# Copy the config file from Unity
cp ../../../Unity/config.txt ./config.txt

# Get the directory of the final results
cd ../../../TestGeneration/FinalResults/${savedirectory}
results_dir="$PWD"
cd ${current_dir}
pwd

# Change the port number inside the new build
sed -i -e 's/(25001)/('$port')/g' ./config.txt

# Some of the variables we assumed to be static from the generation
beamwidth=5
nodes=250
resolution=4
seed=10

# Compute the simulation time
simulationtime=$[trajlength*9]

# Get the folder
folder=${results_dir}/${saveprefix}_MIT_seed${seed}_length${trajlength}_nodes${nodes}_res${resolution}_beamwidth${beamwidth}_totaltime${totaltime}_simtime${simulationtime}_searchtype_${searchtype}_scoretype_${scoretype}

# Get the total number of tests to run
mapcounter=1
totaltests=$(ls $folder/maps | wc -l)

echo "--------------------------------------------------------"
echo "Processing: $folder"
echo "Total tests found: $totaltests"
echo "--------------------------------------------------------"

while [ $mapcounter -le $totaltests ]
do
	echo "Processing: $folder/maps/map$mapcounter"
	echo " "
	
	if [ -z "$controllertype" ]
	then
		declare -a speeds=(-42 -1 -2 2 5 10)
	else
		declare -a speeds=($controllertype)
	fi

	for speed in "${speeds[@]}"
	do

		# Get the current test
		cp $folder/maps/map$mapcounter/test.txt $PWD/test.txt

		# Run the simulator
		./WorldEngine.x86_64 &

		# Get the PID so that I can kill it later
		unity_PID=$!

		# Wait 30 seconds for unity to start
		sleep 20

		# If speed is != 42 then set minsnap to 0, otherwise turn minsnap on and set speed to -1
		if [ $speed -ne -42 ]
		then 
			minsnap=0
		else
			minsnap=1
			speed=-1
		fi

		# Launch the ros file
		roslaunch flightcontroller fly.launch port:="$port" test_location:="$current_dir" save_location:="$current_dir" speed:="$speed" minsnap:="$minsnap" &

		# Get the PID so that I can kill it later
		roslaunch_PID=$!

		# Each test is given ${simulationtime} seconds
		sleep ${simulationtime}

		# Kill the code
		kill -INT $unity_PID
		kill -INT $roslaunch_PID

		# Remove the temporary test
		rm test.txt
		
		# Save the test to the appropriate file
		mv performance.txt $folder/maps/map$mapcounter/performance_speed$speed\_minsnap$minsnap.txt
		mv attitude_thrust_log.txt $folder/maps/map$mapcounter/attitude_thrust_log_speed$speed\_minsnap$minsnap.txt
		mv velocity_log.txt $folder/maps/map$mapcounter/velocity_log_speed$speed\_minsnap$minsnap.txt
		mv angular_rate_log.txt $folder/maps/map$mapcounter/angular_rate_log_speed$speed\_minsnap$minsnap.txt

		# If it is in min snap mode
		if [ $minsnap -ne 0 ]
		then
			mv all_minsnap$minsnap.png $folder/maps/map$mapcounter/all_minsnap$minsnap\_speed$speed.png
			mv sidexz_minsnap$minsnap.png $folder/maps/map$mapcounter/sidexz_minsnap$minsnap\_speed$speed.png
			mv sideyz_minsnap$minsnap.png $folder/maps/map$mapcounter/sideyz_minsnap$minsnap\_speed$speed.png
			mv top_minsnap$minsnap.png $folder/maps/map$mapcounter/top_minsnap$minsnap\_speed$speed.png
		fi

		# Allow 5 seconds for clean up
		sleep 5
	
	# End speed
	done

	# Increment the mapcounter
	((mapcounter++))
done

echo "Done"

# Go back to the original dir
cd $current_dir
cd ..

# Delete the temp files
rm -r ./Build$port

echo Completed Script