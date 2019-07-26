#!/bin/zsh

for drop in 0.5 0.7 0.9
do
    for way in 3 4 5 6 7
    do
        echo Starting Depth $way Drop $drop Nodes 500
        python3 resmonitor.py -M 10g -T 86400 python3 main.py --depth $way --drop $drop --nodes 500
        echo Complete
        echo -----------------------------------------------------------------------------------
    done
done
