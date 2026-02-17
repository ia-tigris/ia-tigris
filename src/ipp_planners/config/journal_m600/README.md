IPP Journal Paper configs

Add `distribution_z(std::discrete_distribution<>({0.5, 0.5}))` to Tigris.cpp

Sample from two altitudes: 
```
double altitude = Tigris::flight_height + 30 * distribution_z(gen);
```

Remove duplicates
```
// Check if x,y distance the same
if (sqrt(pow(node_near->state->as<XYZPsiStateSpace::StateType>()->getX() - node_feasible->state->as<XYZPsiStateSpace::StateType>()->getX(), 2) +
            pow(node_near->state->as<XYZPsiStateSpace::StateType>()->getY() - node_feasible->state->as<XYZPsiStateSpace::StateType>()->getY(), 2)) < TOLERANCE)
{
    // ROS_ERROR_STREAM("Nodes are x,y colocated");
    // print_two_nodes(node_near->state->as<XYZPsiStateSpace::StateType>(), node_feasible->state->as<XYZPsiStateSpace::StateType>(), "Node Near", "Node Feasible");
    continue;
}
```