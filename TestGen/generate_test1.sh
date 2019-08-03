#!/bin/zsh

res=4
nodes=500
for beamwidth in 1
do
    for depth in 3 4 5 6
    do
        echo Starting Depth $depth Beamwidth $beamwidth Nodes $nodes Resolution $res
        python3 -u resmonitor.py -M 10g -T 43200 python3 -u main.py --drone bebop --depth $depth --beamwidth $beamwidth --nodes $nodes --resolution $res 2>&1 | tee Results/details\_nodes$nodes\_res$res\_beamwidth$beamwidth.txt
        echo Complete
        echo -----------------------------------------------------------------------------------
    done
done


res=4
nodes=500
for beamwidth in 5
do
    for depth in 7 8 9 10
    do
        echo Starting Depth $depth Beamwidth $beamwidth Nodes $nodes Resolution $res
        python3 -u resmonitor.py -M 10g -T 43200 python3 -u main.py --drone bebop --depth $depth --beamwidth $beamwidth --nodes $nodes --resolution $res 2>&1 | tee Results/details\_nodes$nodes\_res$res\_beamwidth$beamwidth.txt
        echo Complete
        echo -----------------------------------------------------------------------------------
    done
done
