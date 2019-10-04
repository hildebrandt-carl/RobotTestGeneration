import glob

all_folders = ["./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_waypoint/",
               "./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_constant/"]

system_types = ["waypoint",
                "constant"]

for stype in system_types:
    for folder in all_folders:

        file_location = folder
        file_names = glob.glob(file_location + "maps/map*/performance_" + stype + ".txt")
        print(file_location + "maps/map*/performance_" + stype + ".txt")

        if len(file_names) <= 0:
            print("NO FILES FOUND FOR: " + str(file_location))
            continue

        file_counter = 0
        for file_name in file_names:

            with open(file_name, "r") as f:
                lines = f.readlines()

                for i in range(0, len(lines)-4):
                    if "Goal switch" in lines[i] and "Goal switch" in lines[i + 4]:
                        print("Error in: " + str(file_name))

                    if "Current Goal Position:" in lines[i]:
                        the_line = lines[i]
                        parts = the_line.split(',')
                        if parts[1][1] != "-":
                            print("Error in: " + str(file_name) + " --- " + str(the_line))

