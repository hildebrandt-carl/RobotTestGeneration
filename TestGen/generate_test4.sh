#!/bin/zsh

seed=10

for res in 2 4
do
  for nodes in 1000 2000
  do
    for beamwidth in 25
    do
        for depth in 10
        do
            echo Starting Depth $depth Beamwidth $beamwidth Nodes $nodes Resolution $res
            python3 -u resmonitor.py -M 10g -T 43200 python3 -u main.py --drone mit --depth $depth --beamwidth $beamwidth --nodes $nodes --resolution $res --seed $seed 2>&1 | tee Results/details\_seed$seed\_depth$depth\_nodes$nodes\_res$res\_beamwidth$beamwidth.txt
            echo Complete
            echo -----------------------------------------------------------------------------------
        done
    done
  done
done