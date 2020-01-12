#!/bin/zsh

# need to make the directory
mkdir Results

score="random"
savename="initial"

beamwidth=5
nodes=250
resolution=4
seed=10
totaltime=7200

for searchtype in "random" "maxvel" "kinematic"
do
  for drone in "mit" "anafi"
  do
    for trajectorylength in 3 4 5 6 7 8 9 10 11 12
    do
      simulationtime=$[trajectorylength*9]
      echo $simulationtime

      python3 -u GenerateTests/resmonitor.py -M 8g -T 8500 python3 -u GenerateTests/main.py --drone ${drone} --searchtype ${searchtype} --score ${score} --trajectorylength ${trajectorylength} --beamwidth ${beamwidth} --nodes ${nodes} --resolution ${resolution} --seed ${seed} --totaltime ${totaltime} --simulationtime ${simulationtime} --savename ${savename} 2>&1 | tee ./Results/${savename}_searchtype_${searchtype}_scoretype_${score}_dronetype_${drone}_trajectorylength_${trajectorylength}_beamwidth_${beamwidth}_nodes_${nodes}_resolution_${resolution}_seed_${seed}_totaltime_${totaltime}_simulationtime_${simulationtime}.txt &
    done

    # Make sure this sleep is as long as the resmonitor timeout
    sleep 8505
  done
done
00