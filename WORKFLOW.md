# Work Flow

## Generate a base set of tests

We start by generating the tests. The point of this is to generate the initial sets of tests. This includes the tests for all the search stratergies namely:

* Random (random)
* Approximate Kinematic (maxvel)
* Full Kinematic (kinematic)

You can generate the tests using 1 of 2 available scripts. The scripts which are available alow you to generate either the full set of initial tests, or just a limitied number of test sets. The full set of initial tests lets you view how the number fo valid trajectories change per the trajectory complexity. The limited number of test sets allows you to generate just  the tests required to run the entire tool chain. The scripts are named as follows:

* initial_all_run.sh
* initial_lim_run.sh (recommended)

**NOTE:** These scripts were designed and run on a computer with a 20 core CPU. I recommend changing the number of python scripts launched to be less than or equal to the number of CPU cores you have available.

To run a script you use
```
$ cd ~/RobotTestGeneration/TestGeneration
$ ./initial_lim_runs.sh
or
$ ./initial_all_run.sh
```

All tests will be output into `RobotTestGeneration/TestGeneration/Results` folder. We want to move them to a final folder. Once your tests are done running. You can do this by checking the list of process. Move the results into the final results folder
```
$ mv Results FinalResults
$ cd FinalResults
$ mkdir initial_run_flown
$ mv `ls -A | grep -v initial_run_flown` ./initial_run_flown
```

Feel free to call `initial_run_flown` anything you wish. Just remember it for later as this folder name is required.


## Fly with WorldEngine

The next stage is to use the a modified version of FlightGoogles simulator known as the WorldEngineSimulation. To do that we need simply need to run a script inside the WorldEngineSimulation. The script works by assuming you have saved the initial sets of tests as follows `RobotTestGeneration/TestGeneration/FinalResults/<your_directory>`. You can run the script as follows:

```
$ cd WorldEngineSimulation
$ ./run_mit_25001.sh <your_directory> <search_type> <score_type> <save_prefix> <trajectory_length> <search_time> <controller_type>
```

After having run the initial set of tests the parameters available are:

* your_directory -> initial_run_flown
* search_type -> random; maxvel; kinematic
* score_type -> random
* save_prefix -> initial
* trajectory_length -> 5; 10
* search_time -> (not required) 3600
* controller_type -> (not required - leave blank for all) -42 -2 -1 2 5 10

The `score_type` and `save_prefix` for at this stage of the test generation can only be a single value. They will be used later on. We however want to run the WorldEngineSimulator on all combinations of `search_type` and `trajectory_length`. To make this process faster if your computer is able to handle more than 1 simulation I have provided three scripts so that you can run them in parallel. **NOTE:** you will not be able to run three of the same script at a time as they use static network addresses and will clash. Thus an example of running the simulation in parrallel would be:

```
$ ./run_mit_25001.sh "initial_run_flown" "random" "random" "initial" "5" "3600"
$ ./run_mit_25002.sh "initial_run_flown" "maxvel" "random" "initial" "5" "3600"
$ ./run_mit_25003.sh "initial_run_flown" "kinematic" "random" "initial" "5" "3600"
```

Remember we need to run it on all combinations so after this run is complete dont forget to run the trajectories of length 10.

```
$ ./run_mit_25001.sh "initial_run_flown" "random" "random" "initial" "10" "3600"
$ ./run_mit_25002.sh "initial_run_flown" "maxvel" "random" "initial" "10" "3600"
$ ./run_mit_25003.sh "initial_run_flown" "kinematic" "random" "initial" "10" "3600"
```

The simulators output will be automatically put into the correct folders inside of `RobotTestGeneration/TestGeneration/FinalResults/<your_directory>`.

## Analyzing Results

The next thing we need to do is to parse all the resulting data to get the details from each of the runs. To do that we use the files which can be found in the `TestGeneration/AnalyzeResults` folder.

First we want to parse the resulting data to extract high level metrics from it. We do that using the file `processResults.py`. For each of the test sets we need to extract high level information from them individually. To do that we can run the following commands:

```
maindir="~/RobotTestGeneration/TestGeneration/FinalResults/<your_directory>
maindir="/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/initial2_run_flown/"
$ python3 processResults.py --main_directory ${maindir} --searchtype "random" --scoretype "random" --fileprefix "initial" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "maxvel" --scoretype "random" --fileprefix "initial" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "random" --fileprefix "initial" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "random" --scoretype "random" --fileprefix "initial" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "maxvel" --scoretype "random" --fileprefix "initial" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "random" --fileprefix "initial" --trajectorylength "10" --searchtime "3600"
```





TODO: (NEED TO UPDATE THIS SO I CAN JUST PASS IN COMMANDS)
You have to run
```
$ python3 graphGenerationStatistics.py
```

Now we want to plot the deviation for RQ1)

python3 graphDeviation.py
Make sure RQ1 is uncommented

initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_random
initial_MIT_seed10_length5_nodes250_res4_beamwidth5_totaltime3600_simtime45_searchtype_kinematic_scoretype_random






















TODO:
This section needs major cleanup


## Generating More Stressfull Tests

Once you have completed that we are ready to learn a scoring function to generate new tests for the robot. This file will learn from the results we just generated which types of trajectories are hardest for the quadrotor. For this section we are only going to be working on the the trajectories generated using the kinematic search approach. First we need to generate models for both the length 5 and length 10 trajectories. We can do this by running the command:

```
$ python3 ProcessResults --main_directory ${maindir} --searchtype "kinematic" --scoretype "random --fileprefix "initial" --trajectorylength "5" --saveprefix "len5" --searchtime "3600"
$ python3 ProcessResults --main_directory ${maindir} --searchtype "kinematic" --scoretype "random --fileprefix "initial" --trajectorylength "10" --saveprefix "len10" --searchtime "3600"
```

Using these models we are able to restart the test generation process from before however using our handcrafted scoring functions as well as our learnt scoring functions. This can be done by first going back to the test generation folder and then restarted the test generation process. Generating the tests can be slightly cumbersome and so I have created a script for it. You can run the script as follows

To run a script you use
```
$ cd ~/RobotTestGeneration/TestGeneration
$ ./improved_lim_run.sh <modeldirectory> <model_prefix> <trajectorylength>
```
























RQ2 
(WE WILL BE MISSING THE LEARNT BUT OH WELL)


So for in our case we need to run:
```
$ cd ~/RobotTestGeneration/TestGeneration
$ ./improved_lim_run.sh 
```

This will crate a set of handcrafted tests.

Move the handcrafted tests into the results folder (DONT USE THESE COMMANDS YET) - BASICALLY MOVE EVERYTHING INTO handcrafted_run_flown)

Waypoint controller tests
```
$ ./run_mit_25001.sh "handcrafted_run_flown" "kinematic" "edge" "handcrafted" "5" "-1"
$ ./run_mit_25002.sh "handcrafted_run_flown" "kinematic" "edge90" "handcrafted" "5" "-1"
$ ./run_mit_25003.sh "handcrafted_run_flown" "kinematic" "edge180" "handcrafted" "5" "-1"
$ ./run_mit_25001.sh "handcrafted_run_flown" "kinematic" "edge" "handcrafted" "10" "-1"
$ ./run_mit_25002.sh "handcrafted_run_flown" "kinematic" "edge90" "handcrafted" "10" "-1"
$ ./run_mit_25003.sh "handcrafted_run_flown" "kinematic" "edge180" "handcrafted" "10" "-1"
```

Fixed Velocity Controller Tests
```
$ ./run_mit_25001.sh "handcrafted_run_flown" "kinematic" "edge" "handcrafted" "5" "-42"
$ ./run_mit_25002.sh "handcrafted_run_flown" "kinematic" "edge90" "handcrafted" "5" "-42"
$ ./run_mit_25003.sh "handcrafted_run_flown" "kinematic" "edge180" "handcrafted" "5" "-42"
$ ./run_mit_25001.sh "handcrafted_run_flown" "kinematic" "edge" "handcrafted" "10" "-42"
$ ./run_mit_25002.sh "handcrafted_run_flown" "kinematic" "edge90" "handcrafted" "10" "-42"
$ ./run_mit_25003.sh "handcrafted_run_flown" "kinematic" "edge180" "handcrafted" "10" "-42"
```

Now you need to process them using:
```
maindir="/Users/carlhildebrandt/Dropbox/UVA/Research/Work/RobotTestGeneration/TestGeneration/FinalResults/handcrafted_run_flown/"
maindir="/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/handcrafted_run_flown/"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "edge" --fileprefix "handcrafted" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "edge90" --fileprefix "handcrafted" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "edge180" --fileprefix "handcrafted" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "edge" --fileprefix "handcrafted" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "edge90" --fileprefix "handcrafted" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "edge180" --fileprefix "handcrafted" --trajectorylength "10" --searchtime "3600"
```

You can use the graph deviation file for this:

Plot the deviation (make sure to comment out the right sections)
$ python3 graphDeviation.py



















RQ3

Need to figure out how to generate a learned value for each controller

based on the inital test runs we are interested in two a folders:
initial_MIT_seed10_length5_nodes250_res4_beamwidth5_totaltime3600_simtime45_searchtype_kinematic_scoretype_random
initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_random

We need to generate a model for each of the types in here. To generate a model we run:
```
maindir="~/RobotTestGeneration/TestGeneration/FinalResults/initial_run_flown"
$ python3 FindTrends.py --maindirectory ${maindir} --searchtype "kinematic" --scoretype "random" --fileprefix "initial" --trajectorylength "5" --searchtime "3600" --saveprefix "len5"
$ python3 FindTrends.py --maindirectory ${maindir} --searchtype "kinematic" --scoretype "random" --fileprefix "initial" --trajectorylength "10" --searchtime "3600" --saveprefix "len10"
```

This will produce a set of models in the models directory named:

* len5_speed-1_minsnap0_poly_features.npy && len5_speed-1_minsnap0_regression_mode.npy
* len5_speed-2_minsnap0_poly_features.npy && len5_speed-2_minsnap0_regression_mode.npy
* len5_speed1_minsnap0_poly_features.npy && len5_speed1_minsnap0_regression_mode.npy
* len5_speed2_minsnap0_poly_features.npy && len5_speed2_minsnap0_regression_mode.npy
* len5_speed5_minsnap0_poly_features.npy && len5_speed5_minsnap0_regression_mode.npy
* len5_speed10_minsnap0_poly_features.npy && len5_speed10_minsnap0_regression_mode.npy

Need to figure out how to then make a test set for each controller
Then we generate a test for each of the system types to do that run:
```
./learned_model_run.sh
```

Learn a controller for each of them and then somehow get the thing to fly on it
```
$ ./run_mit_25001.sh "learned_run_flown" "kinematic" "learned" "learned_speed10_minsnap0" "5" "3600" "10"
$ ./run_mit_25002.sh "learned_run_flown" "kinematic" "learned" "learned_speed2_minsnap0" "5" "3600" "2"
$ ./run_mit_25003.sh "learned_run_flown" "kinematic" "learned" "learned_speed-2_minsnap0" "5" "3600" "-2"

$ ./run_mit_25001.sh "learned_run_flown" "kinematic" "learned" "learned_speed-1_minsnap1" "5" "3600" "-42"
$ ./run_mit_25002.sh "learned_run_flown" "kinematic" "learned" "learned_speed-1_minsnap0" "5" "3600" "-1"
$ ./run_mit_25003.sh "learned_run_flown" "kinematic" "learned" "learned_speed5_minsnap0" "5" "3600" "5"

$ ./run_mit_25001.sh "learned_run_flown" "kinematic" "learned" "learned_speed10_minsnap0" "10" "3600" "10"
$ ./run_mit_25002.sh "learned_run_flown" "kinematic" "learned" "learned_speed2_minsnap0" "10" "3600" "2"
$ ./run_mit_25003.sh "learned_run_flown" "kinematic" "learned" "learned_speed-2_minsnap0" "10" "3600" "-2"

$ ./run_mit_25001.sh "learned_run_flown" "kinematic" "learned" "learned_speed-1_minsnap1" "10" "3600" "-42"
$ ./run_mit_25002.sh "learned_run_flown" "kinematic" "learned" "learned_speed-1_minsnap0" "10" "3600" "-1"
$ ./run_mit_25003.sh "learned_run_flown" "kinematic" "learned" "learned_speed5_minsnap0" "10" "3600" "5"
```
./run_mit_25003.sh "learned_run_flown" "kinematic" "learned" "learned_speed5_minsnap0" "10" "3600" "5"
./run_mit_25001.sh "learned_run_flown" "kinematic" "learned" "learned_speed10_minsnap0" "10" "3600" "10"

Now that you have generated all the execution files we need to analyze them to get the performance metrics
```
maindir="/Users/carlhildebrandt/Dropbox/UVA/Research/Work/RobotTestGeneration/TestGeneration/FinalResults/learned_run_flown/"
maindir="/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/learned_run_flown/"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed-1_minsnap0" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed-2_minsnap0" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed2_minsnap0" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed5_minsnap0" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed10_minsnap0" --trajectorylength "5" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed-1_minsnap1" --trajectorylength "5" --searchtime "3600"

$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed-1_minsnap0" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed-2_minsnap0" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed2_minsnap0" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed5_minsnap0" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed10_minsnap0" --trajectorylength "10" --searchtime "3600"
$ python3 processResults.py --main_directory ${maindir} --searchtype "kinematic" --scoretype "learned" --fileprefix "learned_speed-1_minsnap1" --trajectorylength "10" --searchtime "3600"
```