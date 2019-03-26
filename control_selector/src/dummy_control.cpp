#include "dummy_control.h"

void DummyControl::setLaserData(const std::vector<float> &data)
{
    for (size_t i = 0; i<data.size(); i++)
    {
        if ( data[i] < 0.3 )
        {
            obstacle = true;
            break;
        }
    }
}

void DummyControl::getControl(double &v, double &w)
{
    if (obstacle)
    {
        v = 0, w = 0;
    }
    else {
        v = 0.2;
        w = 0;
    }

}


