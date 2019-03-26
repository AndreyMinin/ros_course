#include "voyagercontrol.h"


void VoyagerControl::setLaserData(const std::vector<float> &data)
{
    obstacle = false;
    for (size_t i = 0; i<data.size(); i++)
    {
        if ( data[i] < min_range )
        {
            obstacle = true;
            ROS_WARN_STREAM("OBSTACLE!!!");
            break;
        }
    }
}

//получение управления
void VoyagerControl::getControl(double& v, double& w)
{
    if (obstacle)
    {
        v = 0;
        w = max_omega;
    }
    else
    {
        v = max_vel;
        w = 0;
    }
}
