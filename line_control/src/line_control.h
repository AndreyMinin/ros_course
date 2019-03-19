#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class LineControl
{
public:
    LineControl();
    // секция приватных функций
private:
    //функция вычисления ошибки управления для движения вдоль прямой
    double cross_track_err_line();
    //функция вычисления ошибки управления для движения вдоль окружности
    double cross_track_err_circle();
    /**
     * Функция, которая будет вызвана
     * при получении данных от лазерного дальномера
     */
    void laserCallback(const sensor_msgs::LaserScan& msg);
    /**
     * Функция, которая будет вызвана при
     * получении сообщения с текущем положением робота
     */
    void poseCallback(const nav_msgs::Odometry& msg);
    /**
     * функция обработчик таймера
     */
    void timerCallback(const ros::TimerEvent&);
    // функция публикации ошибки
    void publish_error(double e);
    // секция приватных членов
 private:
    // заданная координата линии, вдоль которой должен двигаться робот
    double line_y;
    double cx, cy, R;
    // заданная скорость движения
    double task_vel;
    // пропрциональный коэффициент регулятора обратной связи
    double prop_factor;
    // интегральный коэффициент регулятора
    double int_factor;
    // дифференциальный коэффициент регулятора
    double diff_factor;
    // интеграл ошибки
    double int_error;
    // старое значение ошибки
    double old_error;
    // минимально допустимое значение расстояния до препятствия
    double min_obstacle_range;
    // флаг наличия препятствия
    bool obstacle;
    // положение робота
    double x, y, theta;

    // объекты, обеспечивающие связь с ros - должны существовать все время жизни приложения
    //  объект NodeHandle, через который создаются подписки и публикаторы
    ros::NodeHandle node;
    // публикатор команд управления
    ros::Publisher cmd_pub;
    //  публикатор текущей ошибки управления
    ros::Publisher err_pub;
    // подписчики
    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;
    ros::Timer timer1;

};

#endif // LINE_CONTROL_H
