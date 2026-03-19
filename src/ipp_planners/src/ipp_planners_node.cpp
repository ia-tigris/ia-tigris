
#include <ros/console.h>
#include "ipp_planners/ipp_planners_node.h"

using namespace ipp;

int main(int argc, char** argv){
    ros::init(argc, argv, "ipp_planners_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    PlannerNode node(nh,pnh);
    return node.run();
}