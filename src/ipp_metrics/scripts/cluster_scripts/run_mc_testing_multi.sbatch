#!/bin/bash
# Number of jobs to run in parallel
#SBATCH -p titan-gpu
#SBATCH -t 1-12:00:00                         # Wall Time
#SBATCH --ntasks-per-node 1                   # Number of cores (adjust as needed)
#SBATCH --cpus-per-task 3
#SBATCH --mem=10G
#SBATCH --job-name=ipp-mc-testing
#SBATCH --array=1-2  # Use the num_jobs variable here
#SBATCH --output=/storage2/datasets/bradym/test_results/mc_out_%A_%a.out
#SBATCH --error=/storage2/datasets/bradym/test_results/mc_error_%A_%a.err


# Combine commands to run after SSH
# RUN_COMMANDS="/ocean/projects/cis220039p/cherieho/map-explore/map_prediction_toolbox/data_factory/cluster_scripts/run_explore.job"
RUN_COMMANDS="/home/bradym/run_mc_testing.job"

# Execute the combined commands in parallel
echo "Starting task number $SLURM_ARRAY_TASK_ID"
bash -c "$RUN_COMMANDS"