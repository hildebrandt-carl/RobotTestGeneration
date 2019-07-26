#!/bin/zsh

node=500
for drop in 0.1 0.3
do
    for way in 3 4 5
    do
        echo Starting Depth $way Drop $drop Nodes $node
        converted=$((100 * $drop))
        python3 -u resmonitor.py -M 10g -T 43200 time python3 -u main.py --depth $way --drop $drop --nodes $node 2>&1 | tee Results/details\_depth$way\_nodes$node\_drop$converted.txt
        echo Complete
        echo -----------------------------------------------------------------------------------
    done
done
