#!/bin/bash
dir="./data/RandA1_500"
result="results_randA1_500_grasp.txt"
for f in "$dir"/*;do 
	name=$f
	l=${name:(-1)}
echo $l
echo $name >> $result
./bin/kemeny -f "$name" --mip_heur >> $result
done
