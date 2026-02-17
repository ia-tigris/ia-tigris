# particle filter params
uint16 target_id
uint16 class_id
uint16 num_particles
float64 target_priority
float64 duration_since_last_propagation
Particle[] particles  # particles
# linear angular control params, for a LinearAngularControlDistribution
float64 linear_acc_mean
float64 linear_acc_std
float64 angular_vel_mean
float64 angular_vel_std
float64 distance_strictness