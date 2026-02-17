# Simple IPP Simulator

This is a simple continuous-space 3D simulator. The environment emulates a agent/UAV flying in <x,y,z> with targets/"ships" on the plane Z=0. 

### Subscribers and Publishers
It subscribes to a topic that publishes a list of waypoints for the UAV, and publishes the following messages:

- current UAV pose <x,y,z,w>
- Ship positions <x,y>
- Sensor measurement <ships that are detected and their corresponding classification <TP, FP, TN, FN>

The corresponding publisher message definitions are included in the package under the `msg/` directory

### Motion model
The motion model for the UAV is currently a simple unicycle model in <x,y> with the propagation on the Z-axis changing step-wise. 

### Setup Environment Parameters
You can set all the environment parameters in the config file found at `config/sim.yaml`

### Add into your workspace
Adding both planner interface and mapping interface:
```
cd your_ws/src
git clone git@github.com:castacks/ipp_simple_sim.git
```

### Build
In your workspace run catkin build and source
```
catkin build
source devel/setup.bash
```

### Launching the Interface Node
The simulation node can be launched with
```bash
roslaunch ipp_simple_sim sim.launch

### Launch the interface Node with other nodes
cd ~/onr_ws/src/onr_tigris/launch/
roslaunch main.launch sim:=true

# separate terminal, visualize in RViz
rviz -d src/ipp_simple_sim/rviz/sim.rviz
```

If you encounter an import error run the following command
```
export PYTHONPATH="${PYTHONPATH}:/home/satrajit/Documents/planner_ws/src/ipp_simple_sim" 
```
