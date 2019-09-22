#!/bin/zsh

# Change the ROS_MASTER_URI to allow multiple version of ROS to run at the same time
ROS_MASTER_URI=http://localhost:11311

# Source the ros workspace
source ROS_WS/devel/setup.zsh

# Used to save the port number
port=25001

# Create a temporary unity folder
cp -r ./Unity/Build ./Build$port

# Go into the directory
cd ./Build$port

# Get current directory
current_dir="$PWD"

# Change the port number inside the new build
sed -i -e 's/(25001)/('$port')/g' ./config.txt

depthcounter=10

for mainfolder in 'FullTest'
do
	for nodescounter in 250
	do
		for rescounter in 4
		do
			for angle in 180
			do
				for beamcounter in 10
				do
					for simtype in 'kinematic'
					do
						for searchtime in 1200
						do
								# Get the folder
								folder=/TestGen/Results/$mainfolder/MIT_seed10\_depth$depthcounter\_nodes$nodescounter\_res$rescounter\_beamwidth$beamcounter\_searchtime$searchtime\_$simtype\_angle$angle

								# Get the total number of tests to run
								mapcounter=1
								totaltests=$(ls ..$folder/maps | wc -l)

								echo "--------------------------------------------------------"
								echo "Processing: $folder"
								echo "Total tests found: $totaltests"
								echo "--------------------------------------------------------"

								while [ $mapcounter -le 45 ]
								do
									echo "Processing: $folder/maps/map$mapcounter"
									echo " "

									for speed in -1 5
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
										roslaunch flightcontroller fly.launch port:="$port" test_location:="$current_dir" save_location:="$current_dir" speed:="$speed" &

										# Get the PID so that I can kill it later
										roslaunch_PID=$!

										# Each test is given 30 seconds
										sleep 90

										# Kill the code
										kill -INT $unity_PID
										kill -INT $roslaunch_PID

										# Remove the temporary test
										rm test.txt
										
										# Save the test to the appropriate file
										if [ $speed -eq -1 ]
										then
											mv performance.txt ..$folder/maps/map$mapcounter/performance_waypoint.txt
											mv angle_log.txt ..$folder/maps/map$mapcounter/angle_log_waypoint.txt
											mv velocity_log.txt ..$folder/maps/map$mapcounter/velocity_log_waypoint.txt
											mv position_log.txt ..$folder/maps/map$mapcounter/position_log_waypoint.txt
										else
											mv performance.txt ..$folder/maps/map$mapcounter/performance_constant.txt
											mv angle_log.txt ..$folder/maps/map$mapcounter/angle_log_constant.txt
											mv velocity_log.txt ..$folder/maps/map$mapcounter/velocity_log_constant.txt
											mv position_log.txt ..$folder/maps/map$mapcounter/position_log_constant.txt
										fi

										# Allow 30 seconds for gezbo to clean up
										sleep 30
									
									# End speed
									done

									# Increment the mapcounter
									((mapcounter++))
								# End mapcounter
								done
						# End searchtime
						done
					# End simtype
					done
				# End beamcounter
				done
			# End depthcounter
			done
		# End rescounter
		done
	# End nodescounter
	done
# End mainfolder
done

# Go back to the original dir
cd ..

# Delete the temp files
rm -r Build$port/

echo Completed Script
