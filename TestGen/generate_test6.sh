#!/bin/zsh

node=500
for drop in 0.5 0.7 0.9
do
    for way in 9 10
    do
        echo Starting Depth $way Drop $drop Nodes $node
        converted=$((100 * $drop))
        python3 -u resmonitor.py -M 10g -T 43200 time python3 -u main.py --depth $way --drop $drop --nodes $node 2>&1 | tee Results/details\_depth$way\_nodes$node\_drop$converted.txt
        echo Complete
        echo -----------------------------------------------------------------------------------
    done
done
