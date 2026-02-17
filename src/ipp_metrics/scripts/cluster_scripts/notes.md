# Create an allocation for interactive

## With gpu
srun -p titan-gpu --gres=gpu:1 -w bender --pty /bin/bash

## No gpu
srun -p titan-gpu -w bender --pty /bin/bash
srun -p titan-gpu --pty /bin/bash

# Then run singularity interactive
singularity instance start \
    -B /storage2/datasets/bradym/test_results \
    ${PROJECT}/singularity_images/ipp-stack.sif \
    ipp-brady

singularity run instance://ipp-brady

start instance
run instance

# To build your singularity image

singularity build --docker-login ipp-stack.sif docker://bradygm/ipp-dev-20-04:latest
singularity build --docker-login ipp-dev.sif docker://bradygm/ipp-github-build:latest # maybe don't need login???


# To submit your batch
sbatch run_mc_testing_multi.sbatch

# Notes
Each time you modify the test folder, change run_mc_testing_multi.sbatch array numbers
run_mc_testing.sh in 3 places the save directory
build the code


