#!/bin/zsh

seed=10
searchtype='maxvel'
drone='mit'
totaltime=28800
simulationtime=90
gentype="waypoint"
for res in 4
do
  for nodes in 250
  do
    for beamwidth in 10
    do
        for depth in 10
        do
            echo Starting Depth $depth Beamwidth $beamwidth Nodes $nodes Resolution $res TotalTime $totaltime Type $searchtype GenerationType $gentype
            python3 -u resmonitor.py -M 8g -T 43200 python3 -u main.py --drone $drone --type $searchtype --depth $depth --beamwidth $beamwidth --nodes $nodes --resolution $res --seed $seed --plotting --totaltime $totaltime --simulationtime $simulationtime --gentype $gentype 2>&1 | tee Results/$drone\_details\_seed$seed\_depth$depth\_nodes$nodes\_res$res\_beamwidth$beamwidth\_totaltime$totaltime\_simulationtime$simulationtime\_$searchtype\_$gentype.txt
            echo Complete
            echo -----------------------------------------------------------------------------------
        done
    done
  done
done