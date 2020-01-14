#!/bin/zsh

# need to make the directory
mkdir Results

modeldirectory="/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalModels/"

searchtype="kinematic"
save="learned"
drone="anafi"
scoretype="learned"

beamwidth=5
nodes=250
resolution=4
seed=10
totaltime=3600
     
for systemtypes in "anafi_sim"
  do

    savename="${save}_${systemtypes}"

    trajectorylength=10
    simulationtime=90
    modelprefix="len${trajectorylength}_${systemtypes}"

    python3 -u GenerateTests/resmonitor.py -M 8g -T 4000 python3 -u GenerateTests/main.py --drone ${drone} --searchtype ${searchtype} --score ${scoretype} --trajectorylength ${trajectorylength} --beamwidth ${beamwidth} --nodes ${nodes} --resolution ${resolution} --seed ${seed} --totaltime ${totaltime} --simulationtime ${simulationtime} --savename ${savename} --modeldirectory ${modeldirectory} --modelprefix ${modelprefix} 2>&1 | tee ./Results/${savename}_searchtype_${searchtype}_scoretype_${scoretype}_dronetype_${drone}_trajectorylength_${trajectorylength}_seed_${seed}_totaltime_${totaltime}_simulationtime_${simulationtime}.txt &
done
