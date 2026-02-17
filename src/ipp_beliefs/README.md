
Demo instructions
```bash
catkin build

source devel/setup.zsh
rosrun ipp_belief demo_node

# separate window
rosrun tf static_transform_publisher 0 0 0 0 0 0 world my_frame 100

# separate window
rviz -d src/ipp_belief/particles.rviz -f my_frame
```