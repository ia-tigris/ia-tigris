/*
* Copyright (c) 2022 Carnegie Mellon University, Author <bradygmoon@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include "ipp_planners/SearchMap.h"
#include <cmath>

/*
Constructor for empty map
*/
SearchMap::SearchMap() { }
/*
Constructor to set values of member variables
*/
SearchMap::SearchMap(
    std::vector<std::vector<double>> polygon_bounds,
    int x_start, int y_start, int x_end, int y_end,
    int map_resolution,
    double confidence_threshold,
    SensorParams sensor_params,
    std::vector<double> initial_map_values,
    std::vector<double> initial_priority_map,
    std::vector<int> initial_sensor_id_map,
    double time_decay_slope,
    double time_decay_floor
    ) {
    this->polygon_bounds = polygon_bounds;
    for (int i = 0; i < polygon_bounds.size(); ++i) {
        this->CGAL_polygon_bounds.push_back(Point(polygon_bounds[i][0], polygon_bounds[i][1]));
    }
    Polygon_2 pgn(this->CGAL_polygon_bounds.begin(), this->CGAL_polygon_bounds.end());
    if (!pgn.is_simple()) {
        // for(int i = 0; i < polygon_bounds.size(); ++i)
        // {
        //     std::cout << "Point " << i << ": " << polygon_bounds[i][0] << ", " << polygon_bounds[i][1] << std::endl;
        // }
        ROS_ERROR("Search Bounds polygon is not simple (edges intersect)! Unable to perform boundary checking.");
    }
    pgn.is_convex() ? this->bounds_convex = true : this->bounds_convex = false;
    if (!this->bounds_convex) {
        // ROS_WARN("Search Bounds polygon is concave.");
    }
    this->x_start = x_start;
    this->y_start = y_start;
    this->x_end = x_end;
    this->y_end = y_end;
    this->map_resolution = map_resolution;
    this->confidence_threshold = confidence_threshold;
    this->sensor_params = sensor_params;
    if (M_PI_2 - this->sensor_params.pitch > this->sensor_params.get_vfov() / 2)
        this->camera_within_footprint = false;
    else
        this->camera_within_footprint = true;

    this->num_rows = static_cast<int>((x_end - x_start) / map_resolution);
    this->num_cols = static_cast<int>((y_end - y_start) / map_resolution);

    if (time_decay_slope > 0 || time_decay_floor < 0 || time_decay_floor > 1)
        throw std::runtime_error("Invalid time decay parameters");
    this->time_decay_slope = time_decay_slope;
    this->time_decay_floor = time_decay_floor;

    this->reset_map(initial_map_values, initial_priority_map, initial_sensor_id_map);
}

/**
 * @brief Reset the map. If initial_map_values is given, it will be used to set the map.
 * 
 * @param initial_map_values 
 */
void SearchMap::reset_map(std::vector<double> initial_map_values = {},
                          std::vector<double> initial_priority_map = {},
                          std::vector<int> initial_sensor_id_map = {}) {
    // set the entire map to be the initial confidence
    if (initial_map_values.empty()) {
        this->map = std::vector<std::vector<double>>(this->num_rows, std::vector<double>(this->num_cols, search_bounds_confidence));
        this->priority = std::vector<std::vector<double>>(this->num_rows, std::vector<double>(this->num_cols, search_bounds_priority));
        this->sensor_model_id = std::vector<std::vector<int>>(this->num_rows, std::vector<int>(this->num_cols, search_bounds_sensor_model));
    } else {
        this->map = this->unflatten_map<double>(initial_map_values, this->num_rows, this->num_cols);
        this->priority = this->unflatten_map<double>(initial_priority_map, this->num_rows, this->num_cols);
        this->sensor_model_id = this->unflatten_map<int>(initial_sensor_id_map, this->num_rows, this->num_cols);
    }

    // set every cell outside the polygon bounds to be 0
    for (int i = 0; i < this->num_rows; ++i)
        for (int j = 0; j < this->num_cols; ++j)
            if (!SearchMap::check_cell_inside_search_bounds(i, j))
                this->map.at(i).at(j) = 0;
}

/**
 * @brief Given a 1D map input, convert it into a 2D map
 *
 * @param flattened_map
 * @param num_rows
 * @param num_cols
 */
template <typename T>
std::vector<std::vector<T>> SearchMap::unflatten_map(std::vector<T> flattened_map, int num_rows, int num_cols) {
    if (flattened_map.size() != num_rows * num_cols)
        throw std::runtime_error("Flattened map size does not match num_rows * num_cols");

    std::vector<std::vector<T>> new_map;
    for (int i = 0; i < num_rows; ++i) {
        std::vector<T> row;
        for (int j = 0; j < num_cols; ++j) {
            row.push_back(flattened_map[i * num_cols + j]);
        }
        new_map.push_back(row);
    }
    return new_map;
}

void SearchMap::update_priors() {
    // for each target polygon prior, check to see if there is a bounds
    for (unsigned int k = 0; k < prior_list.size(); k++) {
        if (prior_list[k].grid_prior_type == planner_map_interfaces::GridPrior::POLYGON_PRIOR) {
            std::vector<std::vector<double>> polygon_bound = gridprior_to_vec(prior_list[k]);
            std::vector<int> bbox = SearchMap::find_bounding_box(polygon_bound, false);
            for (int i=bbox[0]; i <= bbox[2]; ++i) {
                for (int j=bbox[1]; j <= bbox[3]; ++j) {
                    if (i < 0 || i >= this->num_rows || j < 0 || j >= this->num_cols) {
                        continue;
                    }
                    // check if cell lies completely inside polygon prior bounds
                    if (SearchMap::check_cell_inside_poly(i, j, polygon_bound, false)) {
                        this->map[i][j] = prior_list[k].confidence;
                        this->priority[i][j] = prior_list[k].priority;
                        this->sensor_model_id[i][j] = prior_list[k].sensor_model_id;
                    }
                }
            }
        } else if (prior_list[k].grid_prior_type == planner_map_interfaces::GridPrior::LINE_SEG_PRIOR) {
            std::vector<std::vector<double>> polygon_bound = gridprior_to_vec(prior_list[k]);
            for (int i = 0; i < polygon_bound.size()-1; i++) {
                // find all cells that lie on the line segment i to i+1
                SearchMap::fill_cells_on_line(polygon_bound[i], polygon_bound[i+1], prior_list[k].confidence, prior_list[k].priority, prior_list[k].sensor_model_id);
            }
        } else if (prior_list[k].grid_prior_type == planner_map_interfaces::GridPrior::POINT_PRIOR) {
            if (prior_list[k].bounds.points.size() != 1) {
                // ROS_WARN("Point prior has more than one point\n Setting all of them to the same value.");
            }
            for (int i = 0; i < prior_list[k].bounds.points.size(); i++) {
                int index_i = static_cast<int>((prior_list[k].bounds.points[i].x - SearchMap::x_start) / SearchMap::map_resolution);
                int index_j = static_cast<int>((prior_list[k].bounds.points[i].y - SearchMap::y_start) / SearchMap::map_resolution);
                if (index_i < 0 || index_i >= this->num_rows || index_j < 0 || index_j >= this->num_cols) {
                    continue;
                }
                this->map.at(index_i).at(index_j) = prior_list[k].confidence;
                this->priority.at(index_i).at(index_j) = prior_list[k].priority;
                this->sensor_model_id.at(index_i).at(index_j) = prior_list[k].sensor_model_id;
            }
        } else {
            ROS_ERROR("GridPrior type not recognized");
        }
    }
}

std::vector<int> SearchMap::find_bounding_box(std::vector<std::vector<double>> &projected_camera_bounds, bool want_fully_within) {
/*This function is responsible for finding the circumscribing square of a camera projection*/
    double min_x = DBL_MAX;
    double max_x = DBL_MIN;
    double min_y = DBL_MAX;
    double max_y = DBL_MIN;

    for (int i = 0; i < projected_camera_bounds.size(); ++i) {
        double x = projected_camera_bounds[i][0];
        double y = projected_camera_bounds[i][1];
        if (x < min_x)
            min_x = x;
        if (x > max_x)
            max_x = x;
        if (y < min_y)
            min_y = y;
        if (y > max_y)
            max_y = y;
    }

    // convert the coordinates of the bounding box to belief space discretized cell indices
    std::vector<int> bbox;
    if (want_fully_within) {
        int min_r = static_cast<int>((min_x - this->x_start) / this->map_resolution);
        int max_r = static_cast<int>((max_x - this->x_start) / this->map_resolution);
        int min_c = static_cast<int>((min_y - this->y_start) / this->map_resolution);
        int max_c = static_cast<int>((max_y - this->y_start) / this->map_resolution);
        bbox = {min_r, min_c, max_r, max_c};
    } else {
        int min_r = static_cast<int>(std::floor((min_x - this->x_start) / this->map_resolution));
        int max_r = static_cast<int>(std::ceil((max_x - this->x_start) / this->map_resolution));
        int min_c = static_cast<int>(std::floor((min_y - this->y_start) / this->map_resolution));
        int max_c = static_cast<int>(std::ceil((max_y - this->y_start) / this->map_resolution));
        bbox = {min_r, min_c, max_r, max_c};
    }

    if (bbox[0] < 0)
        bbox[0] = 0;
    if (bbox[1] < 0)
        bbox[1] = 0;
    if (bbox[2] >= this->num_rows)
        bbox[2] = this->num_rows - 1;
    if (bbox[3] >= this->num_cols)
        bbox[3] = this->num_cols - 1;

    return bbox;
}


/**
 * @brief Helper function to find the position of a point relative to a line
            > 0: Query point lies on left of the line.
            = 0: Query point lies on the line.
            < 0: Query point lies on right of the line.
 * 
 * @param pt1 
 * @param pt2 
 * @param r 
 * @param c 
 * @return double 
 */
double SearchMap::substitute_point_in_line(std::vector<double> &pt1, std::vector<double> &pt2, int r, int c) {
    return ((c - pt1[1]) * (pt2[0] - pt1[0])) - ((r - pt1[0]) * (pt2[1] - pt1[1]));
}

// Only works for convex polygons
bool SearchMap::check_point_inside_convex_poly(int x, int y, std::vector<std::vector<double>> &projected_camera_bounds) {
    // copy camera bounds into a vector of Points
    // std::vector<Point> polygon_bounds;
    // for (int i = 0; i < projected_camera_bounds.size(); ++i)
    // {
    //     polygon_bounds.push_back(Point(projected_camera_bounds[i][0], projected_camera_bounds[i][1]));
    // }
    // return CGAL::bounded_side_2(polygon_bounds.begin(), polygon_bounds.end(), Point(x, y), K()) == CGAL::ON_BOUNDED_SIDE;

    int num_same_sides_left = 0;
    int num_same_sides_right = 0;
    for (int i = 0; i < projected_camera_bounds.size(); ++i) {
        double point_in_line = SearchMap::substitute_point_in_line(projected_camera_bounds[i],
                                                                   projected_camera_bounds[(i + 1) % projected_camera_bounds.size()], x, y);
        if (point_in_line >= 0)
            num_same_sides_left++;
        if (point_in_line <= 0)
            num_same_sides_right++;
    }

    if (num_same_sides_left != projected_camera_bounds.size() && num_same_sides_right != projected_camera_bounds.size()) {
        return false;
    }

    return true;
}

/**
 * @brief 
 *  winding number algorithm
 *  '''
 *  For a convex polygon, if the sides of the polygon can be considered as a path
 *  from the first vertex, then a query point is said to be inside the polygon if it
 *  lies on the same side of all the line segments making up the path

 *  :params camera_projection -> projected convex polygon of camera bounds on z=0
 *          target_pos -> ground truth target position
 *  '''
 *  To check if a cell lies inside the footprint, we need to check if all four corners of the cell lie within
 *  Four corners:
 *      row*map_resolution + x_start, col*map_resolution + y_start
 *      (row+1)*map_resolution + x_start, col*map_resolution + y_start
 *      row*map_resolution + x_start, (col+1)*map_resolution + y_start
 *      (row+1)*map_resolution + x_start, (col+1) * map_resolution + y_start
 * 
 * @param row 
 * @param col 
 * @param projected_camera_bounds 
 * @param want_fully_within 
 * @return true 
 * @return false 
 */
bool SearchMap::check_cell_inside_convex_poly(int row, int col, std::vector<std::vector<double>> &projected_camera_bounds, bool want_fully_within) {
    /*
    winding number algorithm
    '''
    For a convex polygon, if the sides of the polygon can be considered as a path
    from the first vertex, then a query point is said to be inside the polygon if it
    lies on the same side of all the line segments making up the path

    :params camera_projection -> projected convex polygon of camera bounds on z=0
            target_pos -> ground truth target position
    '''

    To check if a cell lies inside the footprint, we need to check if all four corners of the cell lie within
    Four corners:
        row*map_resolution + x_start, col*map_resolution + y_start
        (row+1)*map_resolution + x_start, col*map_resolution + y_start
        row*map_resolution + x_start, (col+1)*map_resolution + y_start
        (row+1)*map_resolution + x_start, (col+1) * map_resolution + y_start
    */

    int r = row * this->map_resolution + this->x_start;
    int c = col * this->map_resolution + this->y_start;
    if (!check_point_inside_convex_poly(r, c, projected_camera_bounds)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    r = (row + 1) * this->map_resolution + this->x_start;
    if (!check_point_inside_convex_poly(r, c, projected_camera_bounds)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    r = row * this->map_resolution + this->x_start;
    c = (col + 1) * this->map_resolution + this->y_start;
    if (!check_point_inside_convex_poly(r, c, projected_camera_bounds)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    r = (row + 1) * this->map_resolution + this->x_start;
    c = (col + 1) * this->map_resolution + this->y_start;
    if (!check_point_inside_convex_poly(r, c, projected_camera_bounds)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    if (want_fully_within) {
        return true;
    }
    return false;  // no corners were in
}

/**
 * @brief 
 *  Check if a cell is within a polygon, convex or concave. Slower than the convex version. 
 * 
 * @param row 
 * @param col 
 * @param projected_camera_bounds 
 * @param want_fully_within 
 * @return true 
 * @return false 
 */
bool SearchMap::check_cell_inside_poly(int row, int col, std::vector<std::vector<double>> &projected_camera_bounds, bool want_fully_within) {
    int r = row * this->map_resolution + this->x_start;
    int c = col * this->map_resolution + this->y_start;

    std::vector<Point> CGAL_poly;
    for (int i = 0; i < projected_camera_bounds.size(); ++i)
    {
        CGAL_poly.push_back(Point(projected_camera_bounds[i][0], projected_camera_bounds[i][1]));
    }
    // Check if convex
    Polygon_2 pgn(CGAL_poly.begin(), CGAL_poly.end());
    if (!pgn.is_simple()) {
        ROS_ERROR("Prior polygon is not simple (edges intersect)!.");
    }
    bool prior_convex = pgn.is_convex();

    if (!check_point_inside_poly(r, c, projected_camera_bounds, prior_convex, CGAL_poly)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    r = (row + 1) * this->map_resolution + this->x_start;
    if (!check_point_inside_poly(r, c, projected_camera_bounds, prior_convex, CGAL_poly)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    r = row * this->map_resolution + this->x_start;
    c = (col + 1) * this->map_resolution + this->y_start;
    if (!check_point_inside_poly(r, c, projected_camera_bounds, prior_convex, CGAL_poly)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    r = (row + 1) * this->map_resolution + this->x_start;
    c = (col + 1) * this->map_resolution + this->y_start;
    if (!check_point_inside_poly(r, c, projected_camera_bounds, prior_convex, CGAL_poly)) {
        if (want_fully_within)
            return false;
    } else {
        if (!want_fully_within)
            return true;
    }

    if (want_fully_within) {
        return true;
    }
    return false;  // no corners were in
}

// Works for convex or concave
bool SearchMap::check_point_inside_poly(int x,
                                        int y,
                                        std::vector<std::vector<double>> &projected_camera_bounds,
                                        bool prior_convex,
                                        const std::vector<Point> &CGAL_poly)
{
    if (prior_convex) {
        int num_same_sides_left = 0;
        int num_same_sides_right = 0;
        for (int i = 0; i < projected_camera_bounds.size(); ++i) {
            double point_in_line = SearchMap::substitute_point_in_line(projected_camera_bounds[i],
                                                                    projected_camera_bounds[(i + 1) % projected_camera_bounds.size()], x, y);
            if (point_in_line >= 0)
                num_same_sides_left++;
            if (point_in_line <= 0)
                num_same_sides_right++;
        }

        if (num_same_sides_left != projected_camera_bounds.size() && num_same_sides_right != projected_camera_bounds.size()) {
            return false;
        }

        return true;
    }
    else
    {
        return CGAL::bounded_side_2(CGAL_poly.begin(), CGAL_poly.end(), Point(x, y), K()) == CGAL::ON_BOUNDED_SIDE;
    }
}

/**
 * @brief 
 * 
 *   winding number algorithm
 *   '''
 *   For a convex polygon, if the sides of the polygon can be considered as a path
 *   from the first vertex, then a query point is said to be inside the polygon if it
 *   lies on the same side of all the line segments making up the path

 *   :params camera_projection -> projected convex polygon of camera bounds on z=0
 *           target_pos -> ground truth target position
 *   '''

 *   To check if a cell lies inside the footprint, we need to check if all four corners of the cell lie within
 *   Four corners:
 *       row*map_resolution + x_start, col*map_resolution + y_start
 *       (row+1)*map_resolution + x_start, col*map_resolution + y_start
 *       row*map_resolution + x_start, (col+1)*map_resolution + y_start
 *       (row+1)*map_resolution + x_start, (col+1) * map_resolution + y_start
 * @param row 
 * @param col 
 * @param want_fully_within 
 * @return true 
 * @return false 
 */
bool SearchMap::check_cell_inside_search_bounds(int row, int col, bool want_fully_within) {
    bool at_least_one_corner_within = false;

    int r = row * this->map_resolution + this->x_start;
    int c = col * this->map_resolution + this->y_start;
    if (!check_point_inside_search_bounds(r, c)) {
        if (want_fully_within)
            return false;
    } else {
        at_least_one_corner_within = true;
    }

    r = (row + 1) * this->map_resolution + this->x_start;
    if (!check_point_inside_search_bounds(r, c)) {
        if (want_fully_within)
            return false;
    } else {
        at_least_one_corner_within = true;
    }

    r = row * this->map_resolution + this->x_start;
    c = (col + 1) * this->map_resolution + this->y_start;
    if (!check_point_inside_search_bounds(r, c)) {
        if (want_fully_within)
            return false;
    } else {
        at_least_one_corner_within = true;
    }

    r = (row + 1) * this->map_resolution + this->x_start;
    c = (col + 1) * this->map_resolution + this->y_start;
    if (!check_point_inside_search_bounds(r, c)) {
        if (want_fully_within)
            return false;
    } else {
        at_least_one_corner_within = true;
    }

    if (want_fully_within) {
        return true;
    } else {
        if (at_least_one_corner_within) {
            return true;
        }
    }
    return false;
}

bool SearchMap::check_point_inside_search_bounds(int &x, int &y) {
    if (bounds_convex) {
        int num_same_sides_left = 0;
        int num_same_sides_right = 0;
        for (int i = 0; i < this->polygon_bounds.size(); ++i) {
            double point_in_line = SearchMap::substitute_point_in_line(this->polygon_bounds[i],
                                                                    this->polygon_bounds[(i + 1) % this->polygon_bounds.size()], x, y);
            if (point_in_line >= 0)
                num_same_sides_left++;
            if (point_in_line <= 0)
                num_same_sides_right++;
        }

        if (num_same_sides_left != this->polygon_bounds.size() && num_same_sides_right != this->polygon_bounds.size()) {
            return false;
        }

        return true;
    } else {
        if (CGAL::bounded_side_2(this->CGAL_polygon_bounds.begin(),
                                 this->CGAL_polygon_bounds.end(),
                                 Point(x, y), K()) == CGAL::ON_BOUNDED_SIDE) {
            return true;
        } else {
            return false;
        }
    }
}

std::vector<double> SearchMap::find_footprint_centroid(std::vector<std::vector<double>> &projected_camera_bounds)
/*This function finds the centroid of the polygon created by the projection of the camera FOV on the plane z=0*/
{
    std::vector<double> centroid = {0, 0};
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices
    int i = 0;
    for (i = 0; i < projected_camera_bounds.size(); ++i)
    {
        x0 = projected_camera_bounds[i][0];
        y0 = projected_camera_bounds[i][1];
        x1 = projected_camera_bounds[(i + 1) % projected_camera_bounds.size()][0];
        y1 = projected_camera_bounds[(i + 1) % projected_camera_bounds.size()][1];
        a = x0 * y1 - x1 * y0;
        signedArea += a;
        centroid[0] += (x0 + x1) * a;
        centroid[1] += (y0 + y1) * a;
    }

    signedArea *= 0.5;
    centroid[0] /= (6.0 * signedArea);
    centroid[1] /= (6.0 * signedArea);

    return centroid;
}

double SearchMap::calc_reward_and_belief(double &range, double old_belief, double &new_belief, int &sensor_model_id) 
/*This function calculates the reward and belief for a cell based on the range to the cell and the old belief*/
{
    if (old_belief > this->confidence_threshold){
        new_belief = this->sensor_params.tpr(range, sensor_model_id) * old_belief / (this->sensor_params.tpr(range, sensor_model_id) * old_belief + this->sensor_params.fpr(range, sensor_model_id) * (1 - old_belief));
    }
    else{
        new_belief = this->sensor_params.fnr(range, sensor_model_id) * old_belief / (this->sensor_params.fnr(range, sensor_model_id) * old_belief + this->sensor_params.tnr(range, sensor_model_id) * (1 - old_belief));
    }
    // TODO: this line was hacky; turns out we were getting bad probabilities outside [0,1]
    new_belief = std::max(0., std::min(new_belief, 1.0));
    double belief_diff = new_belief - old_belief;

    // if (belief_diff > 0)
    //     return belief_diff * SearchMap::Rs;
    // else
    //     return abs(belief_diff) * SearchMap::Rf;

    // Find change in entropy
    double delta_entropy = calc_entropy(old_belief)-calc_entropy(new_belief);
    if (delta_entropy < 0)
    {
        ROS_ERROR("Delta Entropy is negative");
    }
    if (isnan(delta_entropy)){
        ROS_ERROR("Delta Entropy is NaN");
    }
        
    if (belief_diff > 0)
        return delta_entropy * SearchMap::Rs;
    else
        return delta_entropy * SearchMap::Rf;
}

/**
 * @brief This function creates an estimate of the expected reward within a projection of the camera FOV in a copy of the belief space map
 * 
 * @param projected_camera_bounds 
 * @param current_node 
 * @param bbox 
 * @return double 
 */
double SearchMap::estimate_belief_and_reward(std::vector<std::vector<double>> &projected_camera_bounds, std::vector<double> &current_node, std::vector<int> &bbox)
{
    double total_reward = 0.0;
    for (int i = bbox[0]; i <= bbox[2]; ++i)
    {
        for (int j = bbox[1]; j <= bbox[3]; ++j)
        {
            // check if cell lies completely inside camera footprint
            if (SearchMap::check_cell_inside_convex_poly(i, j, projected_camera_bounds))
            {
                int r = i * this->map_resolution + this->x_start; //TODO this isn't centroid is it?
                int c = j * this->map_resolution + this->y_start;
                // find distance from node pos to centroid of camera footprint
                double range = std::sqrt(std::pow((r - current_node[0]), 2.0) + std::pow((c - current_node[1]), 2.0) + std::pow((0.0 - current_node[2]), 2.0));
                
                double new_belief = 0;
                // Calculate reward and scale by priority
                // ROS_INFO_STREAM("copy_map[" << i << "][" << j << "] = " << this->copy_map[i][j]);
                if (isnan(this->copy_map[i][j]))
                {
                    ROS_ERROR("copy_map[%d][%d] is nan", i, j);
                }
                if (isnan(range))
                {
                    ROS_ERROR("range is nan");
                }
                if (isnan(this->sensor_model_id.at(i).at(j)))
                {
                    ROS_ERROR("sensor_model_id is nan");
                }
                double reward = SearchMap::calc_reward_and_belief(range, this->copy_map.at(i).at(j), new_belief, this->sensor_model_id.at(i).at(j)) * 
                                this->priority.at(i).at(j) *
                                SearchMap::calc_time_reward_decay(current_node[4]);
                if (isnan(reward))
                {
                    ROS_ERROR("Reward is NaN");
                }
                this->copy_map.at(i).at(j) = new_belief;
                
                total_reward += reward;
            }
        }
    }
    return total_reward;
}

static int index_to_key(int &i, int &j, int &num_cols)
{
    return i * num_cols + j;
}

static std::pair<int, int> key_to_index(int &key, int &num_cols)
{
    return std::make_pair(key / num_cols, key % num_cols);
}

/**
 * @brief This function creates an estimate of the expected reward within a projection of the camera FOV in a copy of the belief space map
 * 
 * @param projected_camera_bounds 
 * @param current_node 
 * @param bbox 
 * @return double 
 */
double SearchMap::estimate_belief_and_reward_with_hashmap(TreeNode *new_state, std::vector<std::vector<double>> &projected_camera_bounds, std::vector<double> &current_node, std::vector<int> &bbox)
{
    double total_reward = 0.0;
    int map_key;
    for (int i = bbox[0]; i <= bbox[2]; ++i)
    {
        for (int j = bbox[1]; j <= bbox[3]; ++j)
        {
            // check if cell lies completely inside camera footprint
            if (SearchMap::check_cell_inside_convex_poly(i, j, projected_camera_bounds))
            {
                int r = i * this->map_resolution + this->x_start; //TODO this isn't centroid is it?
                int c = j * this->map_resolution + this->y_start;
                // find distance from node pos to centroid of camera footprint
                double range = std::sqrt(std::pow((r - current_node[0]), 2.0) + std::pow((c - current_node[1]), 2.0) + std::pow((0.0 - current_node[2]), 2.0));
                
                // Calculate reward and scale by priority
                if (isnan(range))
                {
                    ROS_ERROR("range is nan");
                }
                if (isnan(this->sensor_model_id.at(i).at(j)))
                {
                    ROS_ERROR("sensor_model_id is nan");
                }

                double new_belief = 0;
                // Calculate reward and scale by priority
                // Find current belief
                map_key = index_to_key(i, j, this->num_cols);
                double current_belief = SearchMap::get_map_belief_hashmap(new_state, map_key); 
                double reward = SearchMap::calc_reward_and_belief(range, current_belief, new_belief, this->sensor_model_id.at(i).at(j)) *
                                this->priority.at(i).at(j) *
                                SearchMap::calc_time_reward_decay(current_node[4]);
                if (isnan(reward))
                {
                    ROS_ERROR("Reward is NaN");
                }
                new_state->local_search_map_updates[map_key] = new_belief;
                total_reward += reward;
            }
        }
    }
    return total_reward;
}

double SearchMap::estimate_edge_belief_and_reward(std::vector<std::vector<double>> &edgeGeometry, std::vector<int> &bbox, double yaw,
                                                  std::vector<std::vector<double>> projected_camera_bounds, double height_change, 
                                                  double start_height, std::vector<double> end_node, std::vector<std::vector<double>> end_footprint,
                                                  double& time_elapsed)
{
    /*
    center_slope (m1) = yaw angle of the 2 nodes between which we are finding edge reward

    To update belief for given grid cell G we
    1. find perpendicular intercept from top-left coord of G to center line
    2. find length of perpendicular (x)
    3. find local frame nearest pose
    4. find range to optimal (minimum) viewpoint from nearest pose

    Mid-point of base of edge geometry = <x1, y1>
    Center line of edge geometry => (y-y1) = m1*(x-x1)
    Top-left of grid cell = <x2, y2>
    Perpendicular slope of intercept line (m2) = -1/m1

    Intercepts:
        x = (m2x2 - y2 - m1x1 + y1) / (-m1 + m2)
        y = (-m2(m1x1 - y1) + m1(m2x2 - y2)) / (-m1 + m2)
    */
    double total_reward = 0.0;
    double center_slope = tan(yaw);
    double perpendicular_slope = -1 / center_slope;
    double intercept_denom_const = 1 / (-center_slope + perpendicular_slope);
    double edge_start_pose_mid_x = (edgeGeometry[0][0] + edgeGeometry[5][0]) / 2;
    double edge_start_pose_mid_y = (edgeGeometry[0][1] + edgeGeometry[5][1]) / 2;

    double edge_end_pose_mid_x = (edgeGeometry[2][0] + edgeGeometry[3][0]) / 2;
    double edge_end_pose_mid_y = (edgeGeometry[2][1] + edgeGeometry[3][1]) / 2;

    /*To find where end node footprint overlaps edge geometry*/
    std::vector<double> pt1 = {end_footprint[0][0], end_footprint[0][1]};
    std::vector<double> pt2 = {end_footprint[3][0], end_footprint[3][1]};
    double end_point_side = SearchMap::substitute_point_in_line(pt1, pt2, edge_end_pose_mid_x, edge_end_pose_mid_y);

    double width_center_line = std::sqrt(std::pow((edgeGeometry[1][0] - edgeGeometry[4][0]), 2.0) + std::pow((edgeGeometry[1][1] - edgeGeometry[4][1]), 2.0));

    double len_center_line = std::sqrt(std::pow((edge_start_pose_mid_x - edge_end_pose_mid_x), 2.0) + std::pow((edge_start_pose_mid_y - edge_end_pose_mid_y), 2.0));

    double cam_center_origin_offset = 0;
    double optimal_viewing_height = 0.0;

    double edge_angle = atan2(end_footprint[2][1] - end_footprint[3][1], end_footprint[2][0] - end_footprint[3][0]);
    double center_angle = yaw;
    if (edge_angle < center_angle)
    {
        edge_angle += TWO_PI;
    }
    edge_angle -= center_angle;

    double L;
    double tan_bottom_edge_angle = tan(M_PI_2 - this->sensor_params.pitch - (this->sensor_params.get_vfov() / 2));
    std::vector<double> nearest_pose;

    double range = 0.0;

    // check all the cells that could be reached by this camera footprint
    for (int i = bbox[0]; i <= bbox[2]; ++i)
    {
        for (int j = bbox[1]; j <= bbox[3]; ++j)
        {
            if (SearchMap::check_cell_inside_convex_poly(i, j, edgeGeometry))
            {
                int r = i * this->map_resolution + this->x_start;
                int c = j * this->map_resolution + this->y_start;
                // converting grid cell indices to ENU coordinate frame
                double grid_point_side = SearchMap::substitute_point_in_line(pt1, pt2, r, c);
                if (((grid_point_side >= 0 && end_point_side >= 0) || (grid_point_side < 0 && end_point_side < 0)) && SearchMap::check_cell_inside_convex_poly(i, j, end_footprint)) // && SearchMap::check_cell_inside_convex_poly(i, j, end_footprint)
                    // find distance from node pos to centroid of camera footprint  std::sqrt(std::pow((r - end_node[0]), 2.0) + std::pow((c - end_node[1]), 2.0) + std::pow((0.0 - end_node[2]), 2.0));
                    range = std::sqrt(std::pow((r - end_node[0]), 2.0) + std::pow((c - end_node[1]), 2.0) + std::pow((0.0 - end_node[2]), 2.0));
                else
                {
                    double perpendicular_inter_x;
                    double perpendicular_inter_y;
                    if (center_slope == 0)
                    {
                        perpendicular_inter_x = r;
                        perpendicular_inter_y = edge_start_pose_mid_y;
                    }
                    else
                    {
                        perpendicular_inter_x = (perpendicular_slope * r - c - center_slope * edge_start_pose_mid_x + edge_start_pose_mid_y) * intercept_denom_const;
                        perpendicular_inter_y = (-perpendicular_slope * (center_slope * edge_start_pose_mid_x - edge_start_pose_mid_y) + center_slope * (perpendicular_slope * r - c)) * intercept_denom_const;
                    }
                    double target_dist_to_center_line = std::sqrt(std::pow((r - perpendicular_inter_x), 2.0) + std::pow((c - perpendicular_inter_y), 2.0));
                    double dist_perp_inter2edge_start_pose = std::sqrt(std::pow((perpendicular_inter_x - edge_start_pose_mid_x), 2.0) + std::pow((perpendicular_inter_y - edge_start_pose_mid_y), 2.0));
                    double optimal_viewing_height = height_change * (1 - (dist_perp_inter2edge_start_pose / len_center_line)) + start_height;
                    double bottom_edge_origin_offset = optimal_viewing_height * tan_bottom_edge_angle;
                    if (this->camera_within_footprint)
                        L = optimal_viewing_height * tan(this->sensor_params.get_hfov() / 2);
                    else
                        L = std::sqrt(std::pow((projected_camera_bounds[0][0] - projected_camera_bounds[3][0]), 2.0) + std::pow((projected_camera_bounds[0][1] - projected_camera_bounds[3][1]), 2.0)) / 2;
                    if (target_dist_to_center_line < L)
                    {
                        // Check with intercept with bottom edge
                        if (this->camera_within_footprint)
                            // nearest pose that the agent will be to the cell
                            nearest_pose = {target_dist_to_center_line, cam_center_origin_offset, 0};
                        else
                            nearest_pose = {target_dist_to_center_line, bottom_edge_origin_offset, 0};
                    }
                    else
                    {
                        // Check with intercept with side edge
                        double perp_dist_delta = target_dist_to_center_line - L;
                        double delta_l = perp_dist_delta / tan(edge_angle);
                        if (this->camera_within_footprint)
                            nearest_pose = {target_dist_to_center_line, cam_center_origin_offset + delta_l, 0};
                        else
                            nearest_pose = {target_dist_to_center_line, bottom_edge_origin_offset + delta_l, 0};
                    }
                    // distance from the cell to the nearest pose that the drone will be
                    range = std::sqrt(std::pow((nearest_pose[0] - 0), 2.0) + std::pow((nearest_pose[1] - 0), 2.0) + std::pow(optimal_viewing_height, 2.0));
                }

                double new_belief = 0;
                // Calculate reward and scale by priority
                double reward = SearchMap::calc_reward_and_belief(range, this->copy_map.at(i).at(j), new_belief, this->sensor_model_id.at(i).at(j)) *
                                this->priority.at(i).at(j) *
                                SearchMap::calc_time_reward_decay(time_elapsed);
                // update the belief
                this->copy_map.at(i).at(j) = new_belief;
                
                total_reward += reward;
            }
        }
    }
    return total_reward;
}

double SearchMap::estimate_edge_belief_and_reward_with_hashmap(TreeNode *new_state, std::vector<std::vector<double>> &edgeGeometry, std::vector<int> &bbox, double yaw,
                                                                std::vector<std::vector<double>> projected_camera_bounds, double height_change, 
                                                                double start_height, std::vector<double> end_node, std::vector<std::vector<double>> end_footprint,
                                                                double& time_elapsed)
{
    /*
    center_slope (m1) = yaw angle of the 2 nodes between which we are finding edge reward

    To update belief for given grid cell G we
    1. find perpendicular intercept from top-left coord of G to center line
    2. find length of perpendicular (x)
    3. find local frame nearest pose
    4. find range to optimal (minimum) viewpoint from nearest pose

    Mid-point of base of edge geometry = <x1, y1>
    Center line of edge geometry => (y-y1) = m1*(x-x1)
    Top-left of grid cell = <x2, y2>
    Perpendicular slope of intercept line (m2) = -1/m1

    Intercepts:
        x = (m2x2 - y2 - m1x1 + y1) / (-m1 + m2)
        y = (-m2(m1x1 - y1) + m1(m2x2 - y2)) / (-m1 + m2)
    */
    double total_reward = 0.0;
    double center_slope = tan(yaw);
    double perpendicular_slope = -1 / center_slope;
    double intercept_denom_const = 1 / (-center_slope + perpendicular_slope);
    double edge_start_pose_mid_x = (edgeGeometry[0][0] + edgeGeometry[5][0]) / 2;
    double edge_start_pose_mid_y = (edgeGeometry[0][1] + edgeGeometry[5][1]) / 2;

    double edge_end_pose_mid_x = (edgeGeometry[2][0] + edgeGeometry[3][0]) / 2;
    double edge_end_pose_mid_y = (edgeGeometry[2][1] + edgeGeometry[3][1]) / 2;

    /*To find where end node footprint overlaps edge geometry*/
    std::vector<double> pt1 = {end_footprint[0][0], end_footprint[0][1]};
    std::vector<double> pt2 = {end_footprint[3][0], end_footprint[3][1]};
    double end_point_side = SearchMap::substitute_point_in_line(pt1, pt2, edge_end_pose_mid_x, edge_end_pose_mid_y);

    double width_center_line = std::sqrt(std::pow((edgeGeometry[1][0] - edgeGeometry[4][0]), 2.0) + std::pow((edgeGeometry[1][1] - edgeGeometry[4][1]), 2.0));

    double len_center_line = std::sqrt(std::pow((edge_start_pose_mid_x - edge_end_pose_mid_x), 2.0) + std::pow((edge_start_pose_mid_y - edge_end_pose_mid_y), 2.0));

    double cam_center_origin_offset = 0;
    double optimal_viewing_height = 0.0;

    double edge_angle = atan2(end_footprint[2][1] - end_footprint[3][1], end_footprint[2][0] - end_footprint[3][0]);
    double center_angle = yaw;
    if (edge_angle < center_angle)
    {
        edge_angle += TWO_PI;
    }
    edge_angle -= center_angle;

    double L;
    double tan_bottom_edge_angle = tan(M_PI_2 - this->sensor_params.pitch - (this->sensor_params.get_vfov() / 2));
    std::vector<double> nearest_pose;

    double range = 0.0;

    // check all the cells that could be reached by this camera footprint
    for (int i = bbox[0]; i <= bbox[2]; ++i)
    {
        for (int j = bbox[1]; j <= bbox[3]; ++j)
        {
            if (SearchMap::check_cell_inside_convex_poly(i, j, edgeGeometry))
            {
                int r = i * this->map_resolution + this->x_start;
                int c = j * this->map_resolution + this->y_start;
                // converting grid cell indices to ENU coordinate frame
                double grid_point_side = SearchMap::substitute_point_in_line(pt1, pt2, r, c);
                if (((grid_point_side >= 0 && end_point_side >= 0) || (grid_point_side < 0 && end_point_side < 0)) && SearchMap::check_cell_inside_convex_poly(i, j, end_footprint)) // && SearchMap::check_cell_inside_convex_poly(i, j, end_footprint)
                    // find distance from node pos to centroid of camera footprint  std::sqrt(std::pow((r - end_node[0]), 2.0) + std::pow((c - end_node[1]), 2.0) + std::pow((0.0 - end_node[2]), 2.0));
                    range = std::sqrt(std::pow((r - end_node[0]), 2.0) + std::pow((c - end_node[1]), 2.0) + std::pow((0.0 - end_node[2]), 2.0));
                else
                {
                    double perpendicular_inter_x;
                    double perpendicular_inter_y;
                    if (center_slope == 0)
                    {
                        perpendicular_inter_x = r;
                        perpendicular_inter_y = edge_start_pose_mid_y;
                    }
                    else
                    {
                        perpendicular_inter_x = (perpendicular_slope * r - c - center_slope * edge_start_pose_mid_x + edge_start_pose_mid_y) * intercept_denom_const;
                        perpendicular_inter_y = (-perpendicular_slope * (center_slope * edge_start_pose_mid_x - edge_start_pose_mid_y) + center_slope * (perpendicular_slope * r - c)) * intercept_denom_const;
                    }
                    double target_dist_to_center_line = std::sqrt(std::pow((r - perpendicular_inter_x), 2.0) + std::pow((c - perpendicular_inter_y), 2.0));
                    double dist_perp_inter2edge_start_pose = std::sqrt(std::pow((perpendicular_inter_x - edge_start_pose_mid_x), 2.0) + std::pow((perpendicular_inter_y - edge_start_pose_mid_y), 2.0));
                    double optimal_viewing_height = height_change * (1 - (dist_perp_inter2edge_start_pose / len_center_line)) + start_height;
                    double bottom_edge_origin_offset = optimal_viewing_height * tan_bottom_edge_angle;
                    if (this->camera_within_footprint)
                        L = optimal_viewing_height * tan(this->sensor_params.get_hfov() / 2);
                    else
                        L = std::sqrt(std::pow((projected_camera_bounds[0][0] - projected_camera_bounds[3][0]), 2.0) + std::pow((projected_camera_bounds[0][1] - projected_camera_bounds[3][1]), 2.0)) / 2;
                    if (target_dist_to_center_line < L)
                    {
                        // Check with intercept with bottom edge
                        if (this->camera_within_footprint)
                            // nearest pose that the agent will be to the cell
                            nearest_pose = {target_dist_to_center_line, cam_center_origin_offset, 0};
                        else
                            nearest_pose = {target_dist_to_center_line, bottom_edge_origin_offset, 0};
                    }
                    else
                    {
                        // Check with intercept with side edge
                        double perp_dist_delta = target_dist_to_center_line - L;
                        double delta_l = perp_dist_delta / tan(edge_angle);
                        if (this->camera_within_footprint)
                            nearest_pose = {target_dist_to_center_line, cam_center_origin_offset + delta_l, 0};
                        else
                            nearest_pose = {target_dist_to_center_line, bottom_edge_origin_offset + delta_l, 0};
                    }
                    // distance from the cell to the nearest pose that the drone will be
                    range = std::sqrt(std::pow((nearest_pose[0] - 0), 2.0) + std::pow((nearest_pose[1] - 0), 2.0) + std::pow(optimal_viewing_height, 2.0));
                }

                double new_belief = 0;
                // Calculate reward and scale by priority
                // Find current belief
                int map_key = index_to_key(i, j, this->num_cols);
                double current_belief = 0;
                if (new_state->parent != NULL)
                    current_belief = SearchMap::get_map_belief_hashmap(new_state->parent, map_key);
                else
                    current_belief = SearchMap::get_map_belief_hashmap(new_state, map_key); // For the case of the root node, but it shouldn't have an edge
                double reward = SearchMap::calc_reward_and_belief(range, current_belief, new_belief, this->sensor_model_id.at(i).at(j)) *
                                this->priority.at(i).at(j) *
                                SearchMap::calc_time_reward_decay(time_elapsed);
                // update the belief
                new_state->local_search_map_updates[map_key] = new_belief;
                
                total_reward += reward;
            }
        }
    }
    return total_reward;
}

double SearchMap::get_map_belief_hashmap(TreeNode *node, int &map_key)
{
    std::unordered_map<int,double>::const_iterator itr;
    while(node != NULL)
    {
        itr = node->local_search_map_updates.find(map_key);

        if(itr==node->local_search_map_updates.end())
        {
            // Check the next node
            node = node->parent;
        }
        else
        {
            return itr->second;
        }
    }
    // Cell has not been updated yet. Use map value. 
    std::pair<int,int> i_j = key_to_index(map_key, this->num_cols);
    return this->map.at(i_j.first).at(i_j.second);
}

std::vector<std::vector<double>> SearchMap::find_edge_geometry(std::vector<double> edge_start_pose, std::vector<double> edge_end_pose, double max_range)
{ // start_footprint[1], start_footprint[2]
    std::vector<std::vector<double>> q_rotated = rotated_camera_fov(this->sensor_params, /*roll*/ 0.0, /*pitch*/ this->sensor_params.pitch, /*yaw*/ edge_start_pose[3]);
    std::vector<std::vector<double>> start_footprint = project_camera_bounds_to_plane(edge_start_pose, q_rotated, max_range);
    std::vector<std::vector<double>> end_footprint = project_camera_bounds_to_plane(edge_end_pose, q_rotated, max_range);
    std::vector<std::vector<double>> edgeGeometry = {start_footprint[0], start_footprint[1], end_footprint[1], end_footprint[2], start_footprint[2], start_footprint[3]};
    return edgeGeometry;
}


/**
 * @brief Given a desired end-state, traverse up the tree until it hits the root.
 * For each edge between nodes, calculate the expected information gain. 
 * 
 * @param new_state 
 * @param include_edge 
 * @param speed 
 * @return double 
 */
double SearchMap::search_information_gain(TreeNode *new_state, bool include_edge, double speed) // TreeNode* new_state
{
    auto start_time = ompl::time::now();

    // [ {X,Y,Z,Psi}, {X,Y,Z,Psi}, ... ]
    std::vector<std::vector<double>> path_to_root;// list of XYZPsi drone poses

    this->copy_map = this->map; // a copy of the map to perform calculations on

    auto *current_state = new_state;

    int edge_counter = 0; // count how many edges

    // [ {X,Y,Z,Psi}, {X,Y,Z,Psi}, ... ]
    std::vector<std::vector<double>> edge_coords; // start and end of edge, iff include_edge=True

    auto *node_pt = current_state->state->as<XYZPsiStateSpace::StateType>();
    std::vector<double> current_pos = {node_pt->getX(), node_pt->getY(), node_pt->getZ(), node_pt->getPsi(), current_state->cost.value()/speed};
    path_to_root.push_back(current_pos);
    if (current_state->straight_edge_start_pose.size() == 4 && current_state->straight_edge_end_pose.size() == 4 && include_edge)
    {
        edge_counter++;
        edge_coords.push_back(current_state->straight_edge_end_pose);
        edge_coords.push_back(current_state->straight_edge_start_pose);
    }
    else
    {
        // filling dummy values to keep in sync
        edge_coords.push_back({0.0});
        edge_coords.push_back({0.0});
    }

    while (current_state->parent != nullptr)
    {
        // traverse tree till parent
        current_state = current_state->parent;
        node_pt = current_state->state->as<XYZPsiStateSpace::StateType>();
        current_pos[0] = node_pt->getX();
        current_pos[1] = node_pt->getY();
        current_pos[2] = node_pt->getZ();
        current_pos[3] = node_pt->getPsi();
        current_pos[4] = current_state->cost.value()/speed;
        path_to_root.push_back(current_pos);

        if (include_edge && current_state->straight_edge_start_pose.size() == 4 && current_state->straight_edge_end_pose.size() == 4)
        {
            edge_counter++;
            edge_coords.push_back(current_state->straight_edge_end_pose);
            edge_coords.push_back(current_state->straight_edge_start_pose);
        }

        else
        {
            // filling dummy values to keep in sync
            edge_coords.push_back({0.0});
            edge_coords.push_back({0.0});
        }
    }

    double path_reward = 0.0; // reward of vertices only
    double edge_reward = 0.0; // reward of edges

    int counter = 0;
    int edge_coords_idx = edge_coords.size() - 1;
    int edge_itr = 0;

    // traverse backwards, this goes from the root to the end
    for (int i = path_to_root.size() - 1; i >= 0; --i)
    {
        // find the sensor footprint
        std::vector<std::vector<double>> q_rotated = rotated_camera_fov(this->sensor_params, /*roll*/ 0.0, /*sensor_params.pitch*/ this->sensor_params.pitch, /*yaw*/ path_to_root[i][3]);
        // {X, Y, Z}
        std::vector<double> node_pos = {path_to_root[i][0], path_to_root[i][1], path_to_root[i][2]};
        // [{x,y}, {x,y}, ...]
        std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, this->sensor_params.highest_max_range);
        // gets the cell indices of the bounding box around the projected camera bounds
        // {min r, min c, max r, max c}
        std::vector<int> bbox = SearchMap::find_bounding_box(projected_camera_bounds);

        // update belief of the node and calculate reward for the node, for just the vertex
        path_reward += SearchMap::estimate_belief_and_reward(projected_camera_bounds, path_to_root[i], bbox);

        counter++;
        if (include_edge && edge_coords_idx >= 1)
        {
            if (edge_coords[edge_coords_idx].size() == 4 && edge_coords[edge_coords_idx - 1].size() == 4)
            {
                edge_itr++;
                double height_change = edge_coords[edge_coords_idx - 1][2] - edge_coords[edge_coords_idx][2]; // end height - start height
                // find edge bounding box. Makes a quadilateral, only 3 points because the last point is the same as the first
                // [ {x_0, y_0}, {x_1, y_1}, {x_2, y_2} ]
                std::vector<std::vector<double>> edgeGeometry = SearchMap::find_edge_geometry(edge_coords[edge_coords_idx], edge_coords[edge_coords_idx - 1], this->sensor_params.highest_max_range);
                std::vector<int> edge_bbox = SearchMap::find_bounding_box(edgeGeometry);
                // update map with edge estimate and calculate reward
                std::vector<std::vector<double>> end_footprint_q_rotated = rotated_camera_fov(this->sensor_params, /*roll*/ 0.0, /*pitch*/ this->sensor_params.pitch, /*yaw*/ edge_coords[edge_coords_idx - 1][3]);
                std::vector<std::vector<double>> end_footprint = project_camera_bounds_to_plane(edge_coords[edge_coords_idx - 1], end_footprint_q_rotated, this->sensor_params.highest_max_range);
                edge_reward += SearchMap::estimate_edge_belief_and_reward(edgeGeometry, edge_bbox, edge_coords[edge_coords_idx][3],
                                                                          projected_camera_bounds, height_change, edge_coords[edge_coords_idx][2],
                                                                          edge_coords[edge_coords_idx - 1], end_footprint, path_to_root[i][4]);
            }
            edge_coords_idx -= 2; // removes start and end edge
        }
    }
    double final_time = ompl::time::seconds(ompl::time::now() - start_time);
    // std::cout << "The path reward is ::" << path_reward << std::endl;
    // std::cout << "The edge reward is ::" << edge_reward << std::endl;
    // std::cout << "The total reward is ::" << path_reward + edge_reward << std::endl;
    // std::cout << "Time to compute old is :: " << final_time << std::endl;
    return path_reward + edge_reward;
}

/**
 * @brief Given a desired end-state, traverse up the tree until it hits the root.
 * For each edge between nodes, calculate the expected information gain. 
 * 
 * @param new_state 
 * @param include_edge 
 * @param speed 
 * @return double 
 */
double SearchMap::search_information_gain_with_hashmaps(TreeNode *new_state, bool include_edge, double speed) // TreeNode* new_state
{
    //TODO come back and clean this up
    auto start_time = ompl::time::now();

    // [ {X,Y,Z,Psi}, {X,Y,Z,Psi}, ... ]
    std::vector<std::vector<double>> path_to_root;// list of XYZPsi drone poses


    int edge_counter = 0; // count how many edges

    // [ {X,Y,Z,Psi}, {X,Y,Z,Psi}, ... ]
    std::vector<std::vector<double>> edge_coords; // start and end of edge, iff include_edge=True

    auto *node_pt = new_state->state->as<XYZPsiStateSpace::StateType>();
    assert(speed > 0.0);
    std::vector<double> current_pos = {node_pt->getX(), node_pt->getY(), node_pt->getZ(), node_pt->getPsi(), new_state->cost.value()/speed};
    if (new_state->straight_edge_start_pose.size() == 4 && new_state->straight_edge_end_pose.size() == 4 && include_edge)
    {
        edge_counter++;
        edge_coords.push_back(new_state->straight_edge_end_pose);
        edge_coords.push_back(new_state->straight_edge_start_pose);
    }
    else
    {
        // filling dummy values to keep in sync
        edge_coords.push_back({0.0});
        edge_coords.push_back({0.0});
    }

    double path_reward = 0.0; // reward of vertices only
    double edge_reward = 0.0; // reward of edges
    double parent_reward = 0.0; // reward of parent
    if (new_state->parent != nullptr)
    {
        parent_reward = new_state->parent->search_info;
    }

    int counter = 0;
    int edge_coords_idx = edge_coords.size() - 1;
    int edge_itr = 0;

    // find the sensor footprint
    std::vector<std::vector<double>> q_rotated = rotated_camera_fov(this->sensor_params, /*roll*/ 0.0, /*sensor_params.pitch*/ this->sensor_params.pitch, /*yaw*/ current_pos[3]);
    // {X, Y, Z}
    std::vector<double> node_pos = {current_pos[0], current_pos[1], current_pos[2]};
    // [{x,y}, {x,y}, ...]
    std::vector<std::vector<double>> projected_camera_bounds = project_camera_bounds_to_plane(node_pos, q_rotated, this->sensor_params.highest_max_range);
    // gets the cell indices of the bounding box around the projected camera bounds
    // {min r, min c, max r, max c}
    std::vector<int> bbox = SearchMap::find_bounding_box(projected_camera_bounds);
    
    if (include_edge && edge_coords_idx >= 1)
    {
        if (edge_coords[edge_coords_idx].size() == 4 && edge_coords[edge_coords_idx - 1].size() == 4)
        {
            edge_itr++;
            double height_change = edge_coords[edge_coords_idx - 1][2] - edge_coords[edge_coords_idx][2]; // end height - start height
            // find edge bounding box. Makes a quadilateral, only 3 points because the last point is the same as the first
            // [ {x_0, y_0}, {x_1, y_1}, {x_2, y_2} ]
            std::vector<std::vector<double>> edgeGeometry = SearchMap::find_edge_geometry(edge_coords[edge_coords_idx], edge_coords[edge_coords_idx - 1], this->sensor_params.highest_max_range);
            std::vector<int> edge_bbox = SearchMap::find_bounding_box(edgeGeometry);
            // update map with edge estimate and calculate reward
            std::vector<std::vector<double>> end_footprint_q_rotated = rotated_camera_fov(this->sensor_params, /*roll*/ 0.0, /*pitch*/ this->sensor_params.pitch, /*yaw*/ edge_coords[edge_coords_idx - 1][3]);
            std::vector<std::vector<double>> end_footprint = project_camera_bounds_to_plane(edge_coords[edge_coords_idx - 1], end_footprint_q_rotated, this->sensor_params.highest_max_range);
            edge_reward += SearchMap::estimate_edge_belief_and_reward_with_hashmap(new_state, edgeGeometry, edge_bbox, edge_coords[edge_coords_idx][3],
                                                                                projected_camera_bounds, height_change, edge_coords[edge_coords_idx][2],
                                                                                edge_coords[edge_coords_idx - 1], end_footprint, current_pos[4]);
        }
    }
    // update belief of the node and calculate reward for the node, for just the vertex
    path_reward += SearchMap::estimate_belief_and_reward_with_hashmap(new_state, projected_camera_bounds, current_pos, bbox); //TODO check self for updates

    counter++;
    double final_time = ompl::time::seconds(ompl::time::now() - start_time);
    // std::cout << "The path reward is ::" << path_reward << std::endl;
    // std::cout << "The edge reward is ::" << edge_reward << std::endl;
    // std::cout << "Parent reward is ::" << parent_reward << std::endl;
    // std::cout << "The total reward is ::" << path_reward + edge_reward + new_state->parent->information<< std::endl;
    // std::cout << "Time to compute is :: " << final_time << std::endl;
    new_state->search_info = path_reward + edge_reward + parent_reward;
    return path_reward + edge_reward + parent_reward;
}

std::vector<std::vector<double>> SearchMap::gridprior_to_vec(planner_map_interfaces::GridPrior& grid_prior)
{
    std::vector<std::vector<double>> prior_vec;
    for (int k = 0; k < grid_prior.bounds.points.size(); k++)
    {
        std::vector<double> prior = {grid_prior.bounds.points[k].x, grid_prior.bounds.points[k].y};
        prior_vec.push_back(prior);
    } 
    return prior_vec;
}

double SearchMap::calc_entropy(double& probability)
{
    if (probability < 0 || probability > 1)
    {
        // ROS_ERROR_STREAM("Probability is not between 0 and 1, got: " << probability);
        return 0.0;
    }
    return (probability == 0 || probability == 1.0) ? 0 :
        -probability*log2(probability) - (1-probability)*log2(1-probability);
}

double SearchMap::calc_time_reward_decay(double& time_elapsed)
{
    assert(time_elapsed >= 0);
    double inflection_point = (time_decay_floor-1)/time_decay_slope;
    if(time_elapsed < inflection_point)
    {
        return time_decay_slope*time_elapsed + 1;
    }
    else
    {
        return time_decay_floor;
    }
}

void SearchMap::fill_cells_on_line(std::vector<double>& p1, std::vector<double>& p2, double confidence, double priority, double sensor_model_id)
{
    int map_x_max = map.size();
    int map_y_max = map[0].size();
    double x1 = p1[0];
    double y1 = p1[1];
    double x2 = p2[0];
    double y2 = p2[1];
    int X = static_cast<int>((x1 - this->x_start) / this->map_resolution);
    int Y = static_cast<int>((y1 - this->y_start) / this->map_resolution);
    if (X >= 0 && X < map_x_max && Y >= 0 && Y < map_y_max)
    {
        this->map[X][Y] = confidence;
        this->priority[X][Y] = priority;
        this->sensor_model_id[X][Y] = sensor_model_id;
    }
    double slope_x = (x2 - x1);
    double slope_y = (y2 - y1);
    int stepX = (slope_x > 0) ? 1 : (slope_x < 0) ? -1 : 0;
    int stepY = (slope_y > 0) ? 1 : (slope_y < 0) ? -1 : 0;
    double map_res_double = this->map_resolution;
    double tMaxX = (stepX > 0) ? (map_res_double - std::fmod(x1, map_res_double))/slope_x : std::fmod(x1, map_res_double)/slope_x;
    double tMaxY = (stepY > 0) ? (map_res_double - std::fmod(y1, map_res_double))/slope_y : std::fmod(y1, map_res_double)/slope_y;
    double tDeltaX = map_res_double/std::abs(slope_x);
    double tDeltaY = map_res_double/std::abs(slope_y);
    while(tMaxX <= 1 || tMaxY <= 1)
    {
        if(tMaxX < tMaxY)
        {
            tMaxX += tDeltaX;
            X += stepX;
        }
        else
        {
            tMaxY += tDeltaY;
            Y += stepY;
        }
        if (X >= 0 && X < map_x_max && Y >= 0 && Y < map_y_max)
        {
            this->map[X][Y] = confidence;
            this->priority[X][Y] = priority;
            this->sensor_model_id[X][Y] = sensor_model_id;
        }
    }
}
