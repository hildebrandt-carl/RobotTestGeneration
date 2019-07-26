#!/bin/zsh

for drop in 0.1 0.3
do
    for way in 8 9
    do
        echo Starting Depth $way Drop $drop Nodes 500
        python3 resmonitor.py -M 10g -T 86400 time python3 main.py --depth $way --drop $drop --nodes 500
        echo Complete
        echo -----------------------------------------------------------------------------------
    done
done
