#!/bin/zsh

# need to make the directory
mkdir Results

searchtype="kinematic"
score="random"
savename="initial"
drone="mit"

beamwidth=5
nodes=250
resolution=4
seed=10
totaltime=3600
simulationtime=90


for searchtype in "random" "maxvel" "kinematic"
do
  for drone in "mit" "anafi"
  do
    trajectorylength=5
    simulationtime=45

    python3 -u GenerateTests/resmonitor.py -M 8g -T 4000 python3 -u GenerateTests/main.py --drone ${drone} --searchtype ${searchtype} --score ${score} --trajectorylength ${trajectorylength} --beamwidth ${beamwidth} --nodes ${nodes} --resolution ${resolution} --seed ${seed} --totaltime ${totaltime} --simulationtime ${simulationtime} --savename ${savename} 2>&1 | tee ./Results/${savename}_searchtype_${searchtype}_scoretype_${score}_dronetype_${drone}_trajectorylength_${trajectorylength}_beamwidth_${beamwidth}_nodes_${nodes}_resolution_${resolution}_seed_${seed}_totaltime_${totaltime}_simulationtime_${simulationtime}.txt &
  done


  for drone in "mit" "anafi"
  do
    trajectorylength=10
    simulationtime=90
    
    python3 -u GenerateTests/resmonitor.py -M 8g -T 4000 python3 -u GenerateTests/main.py --drone ${drone} --searchtype ${searchtype} --score ${score} --trajectorylength ${trajectorylength} --beamwidth ${beamwidth} --nodes ${nodes} --resolution ${resolution} --seed ${seed} --totaltime ${totaltime} --simulationtime ${simulationtime} --savename ${savename} 2>&1 | tee ./Results/${savename}_searchtype_${searchtype}_scoretype_${score}_dronetype_${drone}_trajectorylength_${trajectorylength}_beamwidth_${beamwidth}_nodes_${nodes}_resolution_${resolution}_seed_${seed}_totaltime_${totaltime}_simulationtime_${simulationtime}.txt &
  done
done
