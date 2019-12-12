import glob

folder_home = "/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/initial_run_flown/"

folder = ["initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_random",
        "initial_MIT_seed10_length5_nodes250_res4_beamwidth5_totaltime3600_simtime45_searchtype_maxvel_scoretype_random",
        "initial_MIT_seed10_length5_nodes250_res4_beamwidth5_totaltime3600_simtime45_searchtype_kinematic_scoretype_random",
        "initial_MIT_seed10_length5_nodes250_res4_beamwidth5_totaltime3600_simtime45_searchtype_random_scoretype_random",
        "initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_random_scoretype_random",
        "initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_maxvel_scoretype_random"]

system_types = ["speed-2_minsnap0",
                "speed-1_minsnap0",
                "speed2_minsnap0",
                "speed5_minsnap0",
                "speed10_minsnap0",
                "speed-1_minsnap1",
                "speed-1_minsnap2"]

for stype in system_types:
    for f in folder:
        file_location = folder_home + f
        file_names = glob.glob(file_location + "/maps/map*/performance_" + stype + ".txt")
        print(file_location + "maps/map*/performance_" + stype + ".txt")

        if len(file_names) <= 0:
            print("NO FILES FOUND FOR: " + str(file_location))
            print("")
            continue

        file_counter = 0
        for file_name in file_names:

            with open(file_name, "r") as f:
                lines = f.readlines()

                for i in range(0, len(lines)-4):
                    if "Goal switch" in lines[i] and "Goal switch" in lines[i + 4]:
                        print("-------------------------------------------------")
                        print("Error in: " + str(file_name) + " --- " + str(the_line))
                        print("-------------------------------------------------")

                    if "Current Goal Position:" in lines[i]:
                        the_line = lines[i]
                        parts = the_line.split(',')
                        if parts[1][1] != "-":
                            print("-------------------------------------------------")
                            print("Error in: " + str(file_name) + " --- " + str(the_line))
                            print("-------------------------------------------------")

