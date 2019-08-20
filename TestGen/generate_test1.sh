#!/bin/zsh

seed=10
searchtype='random'
drone='mit'
searchtime=36000
angle=180
for res in 4
do
  for nodes in 3200
  do
    for beamwidth in 10
    do
        for depth in 10
        do
            echo Starting Depth $depth Beamwidth $beamwidth Nodes $nodes Resolution $res SearchTime $searchtime Type $searchtype Angle $angle
            python3 -u resmonitor.py -M 8g -T 43200 python3 -u main.py --drone $drone --type $searchtype --depth $depth --beamwidth $beamwidth --nodes $nodes --resolution $res --seed $seed --plotting --searchtime $searchtime --scoreangle $angle 2>&1 | tee Results/$drone\_details\_seed$seed\_depth$depth\_nodes$nodes\_res$res\_beamwidth$beamwidth\_searchtime$searchtime\_$searchtype\_angle$angle.txt
            echo Complete
            echo -----------------------------------------------------------------------------------
        done
    done
  done
done