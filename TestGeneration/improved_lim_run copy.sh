#!/bin/zsh

# need to make the directory
mkdir Results

# Get the variables passed to the bash script
modeldirectory=$1 
modelprefix=$2
trajectorylength=$3

if [ -z "$modeldirectory" ] || [ -z "$trajectorylength" ]  || [ -z "$modelprefix" ]
then
    echo "Please run this script using './improved_lim_run.sh <modeldirectory> <model_prefix> <trajectorylength>'"
	  exit 1
fi

searchtype="kinematic"
savename="improved"
drone="mit"

beamwidth=5
nodes=250
resolution=4
seed=10
totaltime=3600
simulationtime=$[trajectorylength*9]

for scoretype in "edge" "edge90" "edge180" "learned"
  do

    python3 -u GenerateTests/resmonitor.py -M 8g -T 4000 python3 -u GenerateTests/main.py --drone ${drone} --searchtype ${searchtype} --score ${score} --trajectorylength ${trajectorylength} --beamwidth ${beamwidth} --nodes ${nodes} --resolution ${resolution} --seed ${seed} --totaltime ${totaltime} --simulationtime ${simulationtime} --savename ${savename} --modeldirectory ${modeldirectory} --modelprefix ${modelprefix} 2>&1 | tee ./Results/${savename}_searchtype_${searchtype}_scoretype_${score}_dronetype_${drone}_trajectorylength_${trajectorylength}_beamwidth_${beamwidth}_nodes_${nodes}_resolution_${resolution}_seed_${seed}_totaltime_${totaltime}_simulationtime_${simulationtime}.txt &
 
done
