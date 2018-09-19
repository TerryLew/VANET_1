#!/bin/sh

counter=0
while [ $counter -le 2 ]
do
    echo $counter
    opp_run -m -u Cmdenv -n .:../../src/veins --image-path=../../images -l ../../src/veins "omnetpp"$counter".ini" > /dev/null
    ((counter++))
done
