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
$ cd ~/RobotTestGeneration
$ cd TestGeneration
$ ./initial_lim_runs.sh
or
$ ./initial_all_run.sh
```


You can generate the initial set of tests using the initial test generation script.
```
$ cd ~/RobotTestGeneration
$ cd TestGeneration
$ ./initial_run.sh
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
$ ./run_mit_25001.sh <your_directory> <search_type> <score_type> <save_prefix> <trajectory_length>
```

After having run the initial set of tests the parameters available are:

* your_directory -> initial_run_flown
* search_type -> random; maxvel; kinematic
* score_type -> random
* save_prefix -> initial
* trajectory_length -> 5; 10

The `score_type` and `save_prefix` for at this stage of the test generation can only be a single value. They will be used later on. We however want to run the WorldEngineSimulator on all combinations of `search_type` and `trajectory_length`. To make this process faster if your computer is able to handle more than 1 simulation I have provided three scripts so that you can run them in parallel. **NOTE:** you will not be able to run three of the same script at a time as they use static network addresses and will clash. Thus an example of running the simulation in parrallel would be:

```
$ ./run_mit_25001.sh "initial_run_flown" "random" "random" "initial" "5"
$ ./run_mit_25002.sh "initial_run_flown" "maxvel" "random" "initial" "5"
$ ./run_mit_25003.sh "initial_run_flown" "kinematic" "random" "initial" "5"
```

Remember we need to run it on all combinations so after this run is complete dont forget to run the trajectories of length 10.

```
$ ./run_mit_25001.sh "initial_run_flown" "random" "random" "initial" "10"
$ ./run_mit_25002.sh "initial_run_flown" "maxvel" "random" "initial" "10"
$ ./run_mit_25003.sh "initial_run_flown" "kinematic" "random" "initial" "10"
```

The simulators output will be automatically put into the correct folders inside of `RobotTestGeneration/TestGeneration/FinalResults/<your_directory>`.

## Analyzing Results

