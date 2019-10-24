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

depthcounter=10

for mainfolder in 'PolySameTimeFull'
do
	for nodescounter in 250
	do
		for rescounter in 4
		do
			for beamcounter in 10
			do
				for totaltime in 28800
				do
					for simtime in 90
					do
						for gentype in 'score'
						do
							for controllertype in 'speed-2'
							do
								# Get the folder
								folder=/TestGen/Results/$mainfolder/MIT_seed10\_depth$depthcounter\_nodes$nodescounter\_res$rescounter\_beamwidth$beamcounter\_totaltime$totaltime\_simtime$simtime\_$gentype\_$controllertype

								# Get the total number of tests to run
								mapcounter=155
								totaltests=$(ls ..$folder/maps | wc -l)

								echo "--------------------------------------------------------"
								echo "Processing: $folder"
								echo "Total tests found: $totaltests"
								echo "--------------------------------------------------------"

								while [ $mapcounter -le $totaltests ]
								do
									echo "Processing: $folder/maps/map$mapcounter"
									echo " "

									for speed in -2
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
										mv performance.txt ..$folder/maps/map$mapcounter/performance_speed$speed.txt
										mv angle_log.txt ..$folder/maps/map$mapcounter/angle_log_speed$speed.txt
										mv velocity_log.txt ..$folder/maps/map$mapcounter/velocity_log_speed$speed.txt
										mv position_log.txt ..$folder/maps/map$mapcounter/position_log_speed$speed.txt

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
				done
			done
		done
	done
done

# Go back to the original dir
cd ..

# Delete the temp files
rm -r Build$port/

echo Completed Script














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

depthcounter=10

for mainfolder in 'PolySameTimeFull'
do
	for nodescounter in 250
	do
		for rescounter in 4
		do
			for beamcounter in 10
			do
				for totaltime in 28800
				do
					for simtime in 90
					do
						for gentype in 'score'
						do
							for controllertype in 'speed-1'
							do
								# Get the folder
								folder=/TestGen/Results/$mainfolder/MIT_seed10\_depth$depthcounter\_nodes$nodescounter\_res$rescounter\_beamwidth$beamcounter\_totaltime$totaltime\_simtime$simtime\_$gentype\_$controllertype

								# Get the total number of tests to run
								mapcounter=169
								totaltests=$(ls ..$folder/maps | wc -l)

								echo "--------------------------------------------------------"
								echo "Processing: $folder"
								echo "Total tests found: $totaltests"
								echo "--------------------------------------------------------"

								while [ $mapcounter -le $totaltests ]
								do
									echo "Processing: $folder/maps/map$mapcounter"
									echo " "

									for speed in -1
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
										mv performance.txt ..$folder/maps/map$mapcounter/performance_speed$speed.txt
										mv angle_log.txt ..$folder/maps/map$mapcounter/angle_log_speed$speed.txt
										mv velocity_log.txt ..$folder/maps/map$mapcounter/velocity_log_speed$speed.txt
										mv position_log.txt ..$folder/maps/map$mapcounter/position_log_speed$speed.txt

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
				done
			done
		done
	done
done

# Go back to the original dir
cd ..

# Delete the temp files
rm -r Build$port/

echo Completed Script