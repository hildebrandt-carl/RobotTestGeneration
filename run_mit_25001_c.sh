#!/bin/zsh

# Change the ROS_MASTER_URI to allow multiple version of ROS to run at the same time
ROS_MASTER_URI=http://localhost:11313

# Source the ros workspace
source ROS_WS/devel/setup.zsh

# Used to save the port number
port=25003

# Create a temporary unity folder
cp -r ./Unity/Build ./Build$port

# Go into the directory
cd ./Build$port

# Get current directory
current_dir="$PWD"

# Change the port number inside the new build
sed -i -e 's/(25001)/('$port')/g' ./config.txt

# Variables we can change
depthcounter=10
rescounter=4
beamcounter=10
totaltime=28800
simtime=90
nodescounter=250
mainfolder='PolySameTimeFull3'

for minsnap in 0
do
	for gentype in 'kinematic'
	do
		for controllertype in 'waypoint'
		do
			# Get the folder
			folder=/TestGen/Results/$mainfolder/MIT_seed10\_depth$depthcounter\_nodes$nodescounter\_res$rescounter\_beamwidth$beamcounter\_totaltime$totaltime\_simtime$simtime\_$gentype\_$controllertype

			# Get the total number of tests to run
			mapcounter=1
			totaltests=$(ls ..$folder/maps | wc -l)

			echo "--------------------------------------------------------"
			echo "Processing: $folder"
			echo "Total tests found: $totaltests"
			echo "--------------------------------------------------------"

			while [ $mapcounter -le $totaltests ]
			do
				echo "Processing: $folder/maps/map$mapcounter"
				echo " "

				# If it is in min snap mode
				if [ $minsnap -ne 0 ]
				then
					declare -a speeds=(-1)
				# Otherwise use all speeds
				else
					declare -a speeds=(5 10)
				fi

				for speed in "${speeds[@]}"
				do

					# Get the current test
					cp ..$folder/maps/map$mapcounter/test.txt test.txt

					# Run the simulator
					./WorldEngine.x86_64 &

					# Get the PID so that I can kill it later
					unity_PID=$!

					# Wait 30 seconds for unity to start
					sleep 20

					# Launch the ros file
					roslaunch flightcontroller fly.launch port:="$port" test_location:="$current_dir" save_location:="$current_dir" speed:="$speed" minsnap:="$minsnap" &

					# Get the PID so that I can kill it later
					roslaunch_PID=$!

					# Each test is given 30 seconds
					if [ $minsnap -eq 2 ]
					then
						sleep 270
					else
						sleep 90
					fi

					# Kill the code
					kill -INT $unity_PID
					kill -INT $roslaunch_PID

					# Remove the temporary test
					rm test.txt
					
					# Save the test to the appropriate file
					mv performance.txt ..$folder/maps/map$mapcounter/performance_speed$speed\_minsnap$minsnap.txt
					mv attitude_thrust_log.txt ..$folder/maps/map$mapcounter/attitude_thrust_log_speed$speed\_minsnap$minsnap.txt
					mv velocity_log.txt ..$folder/maps/map$mapcounter/velocity_log_speed$speed\_minsnap$minsnap.txt
					mv angular_rate_log.txt ..$folder/maps/map$mapcounter/angular_rate_log_speed$speed\_minsnap$minsnap.txt


					# If it is in min snap mode
					if [ $minsnap -ne 0 ]
					then
						mv all_minsnap$minsnap.png ..$folder/maps/map$mapcounter/all_minsnap$minsnap\_speed$speed.png
						mv sidexz_minsnap$minsnap.png ..$folder/maps/map$mapcounter/sidexz_minsnap$minsnap\_speed$speed.png
						mv sideyz_minsnap$minsnap.png ..$folder/maps/map$mapcounter/sideyz_minsnap$minsnap\_speed$speed.png
						mv top_minsnap$minsnap.png ..$folder/maps/map$mapcounter/top_minsnap$minsnap\_speed$speed.png
					fi

					# Allow 5 seconds for clean up
					sleep 5
				
				# End speed
				done

				# Increment the mapcounter
				((mapcounter++))
			done
		done
	done
done

# Go back to the original dir
cd ..

# Delete the temp files
rm -r Build$port/

echo Completed Script