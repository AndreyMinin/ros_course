#pragma once

#include <ros/ros.h>
#include "control.h"

class VoyagerControl : public Control
{
private:
    double min_range;
    bool obstacle = false;
    double max_vel;
    double max_omega;

public:
    //установка данных лазера
    void setLaserData(const std::vector<float>& data) override;

    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    void setRobotPose(double x, double y, double theta) override {}

    //получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override { return "Voyager"; }

    VoyagerControl(double range = 1.0, double maxv = 0.5, double maxw = 0.5):
        min_range(range),
        max_vel(maxv),
        max_omega(maxw)
    {
        ROS_DEBUG_STREAM("VoyagerControl constructor");
    }
};

