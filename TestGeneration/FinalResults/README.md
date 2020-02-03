# Final Results

This folder contains the final results which are presented in our work. The results include the following folders:

* **anafi_learned_run_flown**: These contain the tests that were generated using a scoring model for the Anafi quadrotor
* **handcrafted_run_flown**: These are the tests which were created using the handcrafted scoring metrics.
* **initial_run_flown**: These are the initial tests which were created using different kinematic and dynamic models and used no scoring function. 
* **learned_run_flown**: These are the tests created for the flightgoggles controllers using a learned scoring model.

Each of the tests directories lists the search parameters in the name. In specific the initial tests on a certain number of the tests folders were flown. The tests which were flown are:
```
initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime7200_simtime90_searchtype_kinematic_scoretype_random
```

Inside that folder you will find analysis on the flight as well as the raw flight results.