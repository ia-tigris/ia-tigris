/**
 * @file region.hpp
 * @author your name (you@domain.com)
 * @brief represent regions/bounds of a space.
 * @version 0.1
 * @date 2022-02-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ipp_belief_REGION_H
#define ipp_belief_REGION_H

#include <limits>
#include <vector>

#include "state.h"

namespace tracking
{

    struct Point2d
    {
        double x;
        double y;
    };
    // Function to check if a point is inside a quadrilateral
    bool is_inside_quadrilateral(Point2d p, Point2d q1, Point2d q2, Point2d q3, Point2d q4) 
    {
        // Calculate the cross product of two vectors
        auto cross_product = [](Point2d a, Point2d b, Point2d c)
        {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        };

        // Check if the point is on the same side of each quadrilateral edge
        int cp1 = cross_product(q1, q2, p);
        int cp2 = cross_product(q2, q3, p);
        int cp3 = cross_product(q3, q4, p);
        int cp4 = cross_product(q4, q1, p);

        // If the cross products have the same sign, the point is inside the quadrilateral
        if ((cp1 >= 0 && cp2 >= 0 && cp3 >= 0 && cp4 >= 0) || (cp1 <= 0 && cp2 <= 0 && cp3 <= 0 && cp4 <= 0))
        {
            return true;
        }

        return false;
    }

    /**
     * @brief Defines a polygon region in XY space.
     *
     */
    class Polygon
    {
        // code borrowed from
        // https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon
    public:
        std::string name;
        std::vector<Point2d> polygon;

        /**
         * @brief Construct a new Polygon X Y Region object
         *
         * @param points vector of points that represent <x1, y1> <x2, y2> .... We take in std::vector<double> because it may be x y z, while we only care about xy
         */
        Polygon(std::vector<std::vector<double>> &points) : name("Polygon")
        {
            for (auto &point : points)
            {
                polygon.push_back({point[0], point[1]});
            }
        }

        /**
         * @brief Construct a new Polygon X Y Region object
         *
         * @param points vector of points that represent x_1 y_1 x_2 y_2 ... x_n y_n
         */
        Polygon(std::vector<double> &points)
        {
            for (int i = 0; i < points.size(); i += 2)
            {
                // printf("Adding point %f, %f\n", points[i], points[i + 1]);
                polygon.push_back({points[i], points[i + 1]});
            }
        }

        /**
         * @brief Construct a new Polygon XY-space Region object.
         *
         * @param vertices vertices of polygon. vertices must be in order to connect sides
         * of the polygon
         */
        Polygon(std::vector<TargetState> vertices)
        {
            // There must be at least 3 vertices in polygon
            if (vertices.size() < 3)
            {
                throw std::domain_error(
                    "Polygon: polygon must have at least 3 vertices");
            }
            for (auto &v : vertices)
            {
                Point2d p;
                p.x = v.get_x();
                p.y = v.get_y();
                this->polygon.push_back(p);
            }
        }

        // Given three collinear points p, q, r, the function checks if
        // podouble q lies on line segment 'pr'
        bool on_segment(Point2d p, Point2d q, Point2d r) const
        {
            if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
                q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
                return true;
            return false;
        }

        // To find orientation of ordered triplet (p, q, r).
        // The function returns following values
        // 0 --> p, q and r are collinear
        // 1 --> Clockwise
        // 2 --> Counterclockwise
        int orientation(Point2d p, Point2d q, Point2d r) const
        {
            double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

            if (val == 0)
                return 0;             // collinear
            return (val > 0) ? 1 : 2; // clock or counterclock wise
        }

        // The function that returns true if line segment 'p1q1'
        // and 'p2q2' intersect.
        bool do_intersect(Point2d p1, Point2d q1, Point2d p2, Point2d q2) const
        {
            // Find the four orientations needed for general and
            // special cases
            double o1 = orientation(p1, q1, p2);
            double o2 = orientation(p1, q1, q2);
            double o3 = orientation(p2, q2, p1);
            double o4 = orientation(p2, q2, q1);

            // General case
            if (o1 != o2 && o3 != o4)
                return true;

            // Special Cases
            // p1, q1 and p2 are collinear and p2 lies on segment p1q1
            if (o1 == 0 && on_segment(p1, p2, q1))
                return true;

            // p1, q1 and p2 are collinear and q2 lies on segment p1q1
            if (o2 == 0 && on_segment(p1, q2, q1))
                return true;

            // p2, q2 and p1 are collinear and p1 lies on segment p2q2
            if (o3 == 0 && on_segment(p2, p1, q2))
                return true;

            // p2, q2 and q1 are collinear and q1 lies on segment p2q2
            if (o4 == 0 && on_segment(p2, q1, q2))
                return true;

            return false; // Doesn't fall in any of the above cases
        }

        // Returns true if the podouble p lies inside the polygon[] with n vertices
        virtual bool contains(Point2d p) const
        {
            // special case faster function
            if (polygon.size() == 4){
                return is_inside_quadrilateral(p, polygon.at(0), polygon.at(1), polygon.at(2), polygon.at(3));
            }

            // Create a podouble for line segment from p to infinite

            Point2d extreme = {std::numeric_limits<double>::max(), p.y};

            // Count intersections of the above line with sides of polygon
            int count = 0, i = 0;
            do
            {
                int next = (i + 1) % polygon.size();

                // Check if the line segment from 'p' to 'extreme' intersects
                // with the line segment from 'polygon[i]' to 'polygon[next]'
                if (do_intersect(polygon[i], polygon[next], p, extreme))
                {
                    // If the podouble 'p' is collinear with line segment 'i-next',
                    // then check if it lies on segment. If it lies, return true,
                    // otherwise false
                    if (orientation(polygon[i], p, polygon[next]) == 0)
                        return on_segment(polygon[i], p, polygon[next]);

                    count++;
                }
                i = next;
            } while (i != 0);

            // Return true if count is odd, false otherwise
            return count & 1; // Same as (count%2 == 1)
        }

        friend std::ostream &operator<<(std::ostream &out, const Polygon &s)
        {
            out << s.name << ": ";
            for (auto &p : s.polygon)
            {
                out << "(" << p.x << ", " << p.y << "), ";
            }
            out << std::endl;
            return out;
        }
    };

    // class Quadrilateral : public Polygon
    // {
    // public:
    //     void check_valid_quadrilateral(std::vector<Point2d> &polygon) const
    //     {
    //         if (polygon.size() != 4)
    //         {
    //             throw std::domain_error(
    //                 "Quadrilateral: quadrilateral must have 4 vertices");
    //         }
    //     }

    //     // Constructor
    //     Quadrilateral(std::vector<std::vector<double>> &points)
    //         : Polygon(points)
    //     {
    //         this->name = "Quadrilateral";
    //         check_valid_quadrilateral(this->polygon);
    //     }

    //     // Constructor
    //     Quadrilateral(std::vector<double> &points)
    //         : Polygon(points)
    //     {
    //         this->name = "Quadrilateral";
    //         check_valid_quadrilateral(this->polygon);
    //     }

    //     // Returns true if the podouble p lies inside the quadrilateral

    //     virtual bool contains(Point2d p) const override
    //     {
    //         return is_inside_quadrilateral(p, polygon.at(0), polygon.at(1), polygon.at(2), polygon.at(3));
    //     }
    // };
} // namespace tracking
#endif // REGION_H
