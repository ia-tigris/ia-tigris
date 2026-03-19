#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <string>
#include <ros/ros.h>

#include <ros/ros.h>

namespace ros_utils
{
    /**
     * This function returns ROS params for C++ initialization lists.
     * The default ROS nh.getParam() assigns to a variable, which won't work for an initialization list.
     * This version returns the value.
     * This version also explicitly throws an error if the parameter isn't set (e.g. in the roslaunch file).
     * This is better than failing silently.
     *
     * @tparam ParamT
     * @param nh
     * @param param_name
     * @author Andrew Jong 2022
     * @return ParamT
     */
    template <class ParamT>
    ParamT get_param(ros::NodeHandle &nh, std::string param_name)
    {
        ParamT param;
        if (!nh.getParam(param_name, param))
        {
            throw std::runtime_error("Error: no ROS param set for " + nh.getNamespace() + "/" + param_name + " of type <?>. If you're sure the ROSparam is set, check to make sure you're templating this function with the right datatype.");
        }
        return param;
    }

    /**
     * @brief Get the param object, but with a default value if the param isn't set.
     * 
     * @tparam ParamT 
     * @param nh 
     * @param param_name 
     * @param default_value 
     * @return ParamT 
     */
    template <class ParamT>
    ParamT get_param(ros::NodeHandle &nh, std::string param_name, ParamT default_value)
    {
        ParamT param;
        if (!nh.getParam(param_name, param))
        {
            ROS_WARN_STREAM("Could not get " << param_name << " parameter. Using default of " << default_value);
            param = default_value;
        }
        return param;
    }

}

#endif // ROS_UTILS_H
