#include "slope_calculation.h"

int main(int argc, char **argv)
{

    std::cout << " hello world " << std::endl;
    ros::init(argc, argv, "slope_calculation_node");
    ROS_INFO("hhhh");
    SlopeCalculation app;
    app.init();
    app.createRosPubSub();
    ros::spin();
    return 0;
}
