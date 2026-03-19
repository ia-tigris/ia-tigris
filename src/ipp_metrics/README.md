# IPP Metrics

## Generating Simple Sim Metrics

A baseline implementation exists for search requests with static targets that keeps track of

* Search Map 
    * Average of Search Map
    * Average Entropy of Search Map
* Detections
    * IDs of detected targets
    * Pose estimates of detected target

The search map metrics are formatted as CSV while the detections metrics are a pickled dictionary. For the pickled dictionary, the keys are the detected target IDs, while the values are a list of the pose detections. Thus, the number of detections for a given target can be inferred by taking the length of this lists. 

The node implementation iterates through search maps found in the `search_requests/` directory and automatically spins up a simple sim run for the given map. After each test is complete, the corresponding nodes are killed and the metrics are saved to the `sim_test_results/` directory. To run simple sim search metrics tests, run

```
roslaunch metrics simple_sim_metrics.launch
```
