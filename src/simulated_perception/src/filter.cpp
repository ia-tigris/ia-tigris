#include "simulated_perception/filter.h"

double init_speed = 5;

TargetFilter::TargetFilter(const u_int8_t index,
                           const double observed_time,
                           const double heading,
                           const Eigen::Vector2d& first_observed_state,
                           const Eigen::Matrix2d& first_observed_cov)
{
    target_index_ = index;
    last_observed_time_ = observed_time;
    heading_ = heading;

    observed_state << first_observed_state(0), first_observed_state(1), heading;
    observed_cov.block<2,2>(0,0) = first_observed_cov;
    observed_cov(2,2) = 0.0001;
    // initialize as prev state
    prev_state << observed_state(0), observed_state(1), heading, init_speed, 0;
    // ROS_INFO_STREAM("init state is \n" << prev_state);
    prev_cov << observed_cov(0,0), observed_cov(0,1), 0, 0, 0,
                observed_cov(1,0), observed_cov(1,1), 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
    C_mat << 1,0,0,0,0,
             0,1,0,0,0,
             0,0,1,0,0;

    update_state = prev_state;
    update_cov = prev_cov;
    new_observation_get_ = false;
}

TargetFilter::~TargetFilter()
{
    return;
}

void TargetFilter::get_observation(const Eigen::Vector2d& curr_observed_state,
                                   const Eigen::Matrix2d& curr_observed_cov,
                                   const double curr_heading,
                                   const double curr_observed_time)
{
    if (to_reset_)
    {
        ROS_INFO_STREAM("target " << (int)target_index_ << " is recaptured");
        last_observed_time_ = curr_observed_time;
        heading_ = curr_heading;
        observed_state << curr_observed_state(0), curr_observed_state(1), curr_heading;
        observed_cov.block<2,2>(0,0) = curr_observed_cov;
        observed_cov(2,2) = 0.0001;
        prev_state << observed_state(0), observed_state(1), curr_heading, init_speed, 0;
        prev_cov << observed_cov(0,0), observed_cov(0,1), 0, 0, 0,
                    observed_cov(1,0), observed_cov(1,1), 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1;
        C_mat << 1,0,0,0,0,
                 0,1,0,0,0,
                 0,0,1,0,0;
        update_state = prev_state;
        update_cov = prev_cov;
        new_observation_get_ = false;
        to_reset_ = false;
        return;
    }

    observed_state << curr_observed_state(0), curr_observed_state(1), curr_heading;
    observed_cov.block<2,2>(0,0) = curr_observed_cov;
    observed_cov(2,2) = 0.0001;
    heading_ = curr_heading;
    curr_observed_time_ = curr_observed_time;
    dt_ = curr_observed_time_ - last_observed_time_;
    Qt = 0.0001 * Eigen::MatrixXd::Identity(5,5); // set here
    Qt(3,3) = 0.25;
    new_observation_get_ = true;
}

Eigen::Matrix<double, 5, 5> TargetFilter::get_A_matrix()
{
    Eigen::Matrix<double, 5, 5> A_mat = Eigen::MatrixXd::Zero(5,5);
    A_mat(0,2) = - prev_state(3) * sin(prev_state(2));
    A_mat(0,3) = cos(prev_state(2));
    A_mat(1,2) = prev_state(3) * cos(prev_state(2));
    A_mat(1,3) = sin(prev_state(2));
    A_mat(2,4) = 1;
    return A_mat;
}

bool TargetFilter::filtering()
{
    if (!new_observation_get_)
    {return false;}

    Eigen::Matrix<double, 5, 5> A_mat = get_A_matrix();
    // prediction
    Eigen::Matrix<double, 5, 1> fx;
    fx << prev_state(3) * cos(prev_state(2)),
          prev_state(3) * sin(prev_state(2)),
          prev_state(4),
          0,
          0;
    Eigen::Matrix<double, 5, 1> predict_mean = prev_state + dt_ * fx;

    Eigen::Matrix<double, 5, 5> Ft;
    Ft = Eigen::MatrixXd::Identity(5,5) + dt_ * get_A_matrix();
    Eigen::Matrix<double, 5, 5> Vt;
    Vt = dt_ * Eigen::MatrixXd::Identity(5,5);
    Eigen::Matrix<double, 5, 5> predict_cov = Ft * prev_cov * Ft.transpose() + Vt * Qt * Vt.transpose();
    // update, Wt = dgdv is identity here so Wt * R * Wt^T equals to R = observed_cov
    Eigen::MatrixXd Kt = predict_cov * C_mat.transpose() * (C_mat * predict_cov * C_mat.transpose() + observed_cov).inverse();
    update_state = predict_mean + Kt * (observed_state - C_mat * predict_mean);
    update_cov = predict_cov - Kt * C_mat * predict_cov;

    // std::cout << "prev_state is \n" << prev_state << std::endl;
    // std::cout << "prev_cov is \n" << prev_cov << std::endl;
    // std::cout << "predict_mean is \n" << predict_mean << std::endl;
    // std::cout << "predict_cov is \n" << predict_cov << std::endl;
    // std::cout << "observed_state is \n" << observed_state << std::endl;
    // std::cout << "A_mat is \n" << A_mat << std::endl;
    // std::cout << "C_mat is \n" << C_mat << std::endl;
    // std::cout << "observed_cov is \n" << observed_cov << std::endl;
    // std::cout << "Kt is \n" << Kt << std::endl;
    // std::cout << "update_state is \n" << update_state << std::endl;
    // std::cout << "update_cov is \n" << update_cov << std::endl;
    
    prev_state = update_state;
    prev_cov = update_cov;
    last_observed_time_ = curr_observed_time_;
    new_observation_get_ = false;

    return true;
}

u_int8_t TargetFilter::get_index()
{
    return target_index_;
}

double TargetFilter::get_heading()
{
    return heading_;
}

double TargetFilter::get_time()
{
    return last_observed_time_;
}

void TargetFilter::make_to_reset()
{
    to_reset_ = true;
}

bool TargetFilter::ask_if_reset()
{
    return to_reset_;
}


planner_map_interfaces::FilteredTarget TargetFilter::get_filtered_msg()
{
    planner_map_interfaces::FilteredTarget filtered_state;
    filtered_state.header.frame_id = "local_enu";
    filtered_state.header.stamp = ros::Time(get_time());

    double x = update_state(0);
    double y = update_state(1);
    double heading = update_state(2);
    double linear_speed = update_state(3);
    double angular_speed = update_state(4);


    filtered_state.local_id = get_index();
    filtered_state.x = update_state(0);
    filtered_state.y = update_state(1);
    filtered_state.xdot = update_state(3) * cos(update_state(2));
    filtered_state.ydot = update_state(3) * sin(update_state(2));

    Eigen::Matrix<double, 5, 5> orig(update_cov.data());
    Eigen::Matrix4d cov = orig.block<4, 4>(0, 0);
    // Define the state vector in x, y, heading, linear speed
    Eigen::Vector4d state;
    state << x, y, heading, linear_speed;

    // Compute the Jacobian matrix
    Eigen::Matrix<double, 4, 4> j;
    j << 1, 0, -linear_speed * sin(heading), cos(heading),
         0, 1, linear_speed * cos(heading), sin(heading),
         0, 0, 1, 0,
         0, 0, 0, 1;

    // Compute the new covariance matrix in x, y, xdot, ydot
    Eigen::Matrix<double, 4, 4> new_cov = j * cov * j.transpose();;  
    // std::cout << "New covariance matrix: \n" << new_cov << std::endl;


    // std::cout << "update_cov is \n" << update_cov << std::endl;
    for (size_t row = 0; row<4; row++)
    {
        for (size_t col = 0; col < 4; col++)
        {
            size_t cov_index = row * 4 + col;
            filtered_state.covariance[cov_index] = new_cov(row, col);
        }
    }
    return filtered_state;
}

nav_msgs::Odometry TargetFilter::get_odom()
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = "local_enu";
    odom.header.stamp = ros::Time(get_time());

    odom.pose.pose.position.x = update_state(0);
    odom.pose.pose.position.y = update_state(1);

    odom.pose.covariance[0] = update_cov(0,0);
    odom.pose.covariance[1] = update_cov(0,1);
    odom.pose.covariance[6] = update_cov(1,0);
    odom.pose.covariance[7] = update_cov(1,1);
    
    return odom;
}