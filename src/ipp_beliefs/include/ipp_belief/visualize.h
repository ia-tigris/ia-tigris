
#ifndef VISUALIZE_PARTICLES_H
#define VISUALIZE_PARTICLES_H

#define _USE_MATH_DEFINES

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cmath>
#include <string>
#include <vector>

#include "state.h"
#include "trackers.h"

namespace tracking
{

    std_msgs::ColorRGBA make_color(double r, double g, double b, double a = 1.0)
    {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    /**
     * Visualize TargetState particle as triangles.
     *
     */
    class ParticleFilterVisualizer
    {
    protected:
        ros::NodeHandle *nh;
        std::string topic;
        std_msgs::ColorRGBA color;
        ros::Publisher particle_pub;
        double scale;
        double alpha;

    public:
        visualization_msgs::Marker marker;
        std::string local_frame = "local_enu";

        /**
         * @brief Construct a new ParticleFilterVisualizer object with RGBA
         *
         * @param r
         * @param g
         * @param b
         * @param a
         */
        explicit ParticleFilterVisualizer(
            std_msgs::ColorRGBA color, double scale = 5.0, double alpha = 0.1)
            : color(color), scale(scale), alpha(alpha), nh(nullptr)
        {
            // init_marker();
        }

        /**
         * @brief Construct a new ParticleFilterVisualizer object with RGBA
         *
         * @param r
         * @param g
         * @param b
         * @param a
         */
        explicit ParticleFilterVisualizer(
            ros::NodeHandle &nh, std::optional<std::string> topic = std::nullopt,
            std::optional<std_msgs::ColorRGBA> color = std::nullopt, double scale = 1.0)
            : nh(&nh), scale(scale)
        {
            // printf("In ParticleFilterVisualizer full constructor\n");
            // assign topic name
            this->topic = topic.value_or("particle_filter_marker");
            // assign color
            if (color)
            {
                // printf("We have color\n");
                this->color = color.value();
            }
            else
            {
                // printf("Setting color to white as default\n");
                color->r = color->g = color->b = color->a = 1.0f;
            }
            particle_pub = nh.advertise<visualization_msgs::Marker>(this->topic, 0);
        }

        /**
         * Creates a triangle for each particle to add to the triangle list
         *
         */
        void update(const ParticleFilter &pf_tracker, double alpha = 0.1, std::string ns = "propagation_particles")
        {
            // printf("XYSHParticleFilter update called \n");
            marker.points.clear();
            marker.colors.clear();

            init_marker(pf_tracker, alpha, ns);

            for (auto &p : pf_tracker.get_particles())
            {
                add_triangle_marker(p);
            }
        }

        void update_sparse(const ParticleFilter &pf_tracker, double alpha = 0.1, std::string ns = "propagation_particles")
        {
            // printf("XYSHParticleFilter update called \n");
            marker.points.clear();
            marker.colors.clear();

            init_marker(pf_tracker, alpha, ns);
            int particle_counter = 0;
            int sparse_rate = 10;
            for (auto &p : pf_tracker.get_particles())
            {
                if (particle_counter % sparse_rate == 0)
                {
                    add_triangle_marker(p);
                }
                particle_counter += 1;
            }
        }

        visualization_msgs::Marker get_marker()
        {
            return marker;
        }

    protected:
        void init_marker(const ParticleFilter &pf_tracker, double alpha = 0.1, std::string ns_ = "propagation_particles")
        {
            marker.header.frame_id = local_frame;
            marker.header.stamp = ros::Time();
            marker.ns = ns_;
            marker.id = pf_tracker.get_id();
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 5.0;
            marker.color.a = alpha;
            marker.points.clear();
            marker.colors.clear();
        }

        /**
         * @brief Given a particle p, add a triangle to the marker
         *
         * @param p
         */
        void add_triangle_marker(const TargetState &p)
        {
            // std::cout << "Visualizing particle: " << p << std::endl;
            // unpack values
            double centroid_x = p.get_x();
            double centroid_y = p.get_y();
            double speed = p.get_speed();
            double heading = p.get_heading();
            // PREPARE TRIANGLE MARKER
            double triangle_height =
                1.0 * this->scale; // linear_velocity > 1 ? (std::log10(linear_velocity) + 1) : 0.75 * linear_velocity + 0.25;
            double height_length_above_centroid = triangle_height * 0.666666666666667;
            double height_length_below_centroid = triangle_height * 0.333333333333333;
            double base_width = 0.5 * triangle_height;

            // define triangle vertices
            geometry_msgs::Point top_vertex;
            top_vertex.x = centroid_x + std::cos(heading) * height_length_above_centroid;
            top_vertex.y = centroid_y + std::sin(heading) * height_length_above_centroid;

            double bottom_midpoint_x =
                centroid_x - std::cos(heading) * height_length_below_centroid;
            double bottom_midpoint_y =
                centroid_y - std::sin(heading) * height_length_below_centroid;

            geometry_msgs::Point bottom_left_vertex;
            bottom_left_vertex.x =
                bottom_midpoint_x + std::cos(heading + 0.5 * M_PI) * (0.5 * base_width);
            bottom_left_vertex.y =
                bottom_midpoint_y + std::sin(heading + 0.5 * M_PI) * (0.5 * base_width);
            geometry_msgs::Point bottom_right_vertex;
            bottom_right_vertex.x =
                bottom_midpoint_x + std::cos(heading - 0.5 * M_PI) * (0.5 * base_width);
            bottom_right_vertex.y =
                bottom_midpoint_y + std::sin(heading - 0.5 * M_PI) * (0.5 * base_width);

            marker.points.push_back(top_vertex);
            marker.points.push_back(bottom_left_vertex);
            marker.points.push_back(bottom_right_vertex);
            // printf("DEBUG: Triangle points: {top=(%f,%f), left=(%f,%f),
            // right=(%f,%f)}", top_vertex.x, top_vertex.y, bottom_left_vertex.x,
            // bottom_left_vertex.y, bottom_right_vertex.x, bottom_right_vertex.y);

            marker.colors.push_back(this->color);
        }
    };

} // namespace particle_filter_tracker
#endif // VISUALIZE_PARTICLES_H