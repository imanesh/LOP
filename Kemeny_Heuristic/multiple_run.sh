#!/bin/bash
dir="./data/RandB"
for d in "$dir"/*;do 
	jobname=$d
	cat << EOF | qsub
#PBS -N $jobname
#PBS -l ncpus=8
#PBS -l mem=32g
#PBS -W group_list=colony-users
#PBS -M eia2@sfu.ca
##### Queue #####
#PBS -q workq
module load TOOLS/CPLEX/12.5

########## Output File ##########
#PBS -o \$PBS_O_WORKDIR/workspcae_c++/Kemeny_Heuristic/$jobname.txt

########## Error File ##########
#PBS -e \$PBS_O_WORKDIR/workspcae_c++/Kemeny_Heuristic/$jobname.err

##### Change to current working directory #####
cd \$PBS_O_WORKDIR/workspcae_c++/Kemeny_Heuristic

##### Execute Program #####


./bin/kemeny -f $jobname --mip_heur 0

EOF
done
