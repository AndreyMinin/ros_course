#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt16.h"
#include "control.h"
#include "voyagercontrol.h"
#include "dummy_control.h"

/*
 * Доработать проект control_selector, добавив класс движения вдоль стены
 * (правой и левой) в виде наследника абстрактного класса Control
 *  Добавить объекты нового класса в массив controls, обеспечив тем самым возможность
 * их выбора спомощью selectCallback
 *
 * */

// перечисление, задающее константы, соответствующее индексам алгоритмов в массиве
enum ControlEnum
{
    DUMMY,
    VOYAGER,
    // WALLFOLLOWER, // раскомментировать после добавления нового алгоритма
    nControls
};

// объект издатель команд управления
ros::Publisher cmd_pub;
// указатель на текущий алгоритм управления
Control* controlPtr(nullptr);

// массив указателей на доступные алгоритмы из которых мы можем выбрать
Control* controls[nControls];

// функция обработки сообщения по топику '/selector`, управляющего выбором алгоритма
// можно отправить сообщение из консоли
// rostopic pub /selector std_msgs/UInt16 <номер алгоритма начиная с 0>
void selectCallback(const std_msgs::UInt16& msg)
{
    ROS_INFO_STREAM("select callback "<<msg.data);
    if ( msg.data >= nControls )
    {
        ROS_ERROR_STREAM("Wrong algorithm number " << msg.data);
        controlPtr = nullptr;
    }
    else
    {
        controlPtr = controls[msg.data];
        ROS_INFO_STREAM("Select " << controlPtr->getName() << " control");
    }
}


/**
 * Функция, которая будет вызвана
 * при получении данных от лазерного дальномера
 */
void laserCallback(const sensor_msgs::LaserScan& msg)
{
    ROS_DEBUG_STREAM("Laser msg: ");
    if ( controlPtr ) // если указатель не нулевой - вызываем функцию текущего алгоритма
        controlPtr->setLaserData(msg.ranges);
}


/**
 * Функция, которая будет вызвана при
 * получении сообщения с текущим положением робота
 */

void poseCallback(const nav_msgs::Odometry& msg)
{
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  double theta = 2*atan2(msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w);
  ROS_DEBUG_STREAM("Pose msg: x = "<<x<<" y = "<<y<<" theta = "<<theta );
  if ( controlPtr ) // если указатель не нулевой - вызываем функцию текущего алгоритма
      controlPtr->setRobotPose(x, y, theta);
}

/*
 * функция обработчик таймера
 */
void timerCallback(const ros::TimerEvent&)
{  
    ROS_DEBUG_STREAM("on timer ");
    // сообщение с помощью которого задается
    // управление угловой и линейной скоростью
	geometry_msgs::Twist cmd;
    if ( controlPtr )
    {
        controlPtr->getControl(cmd.linear.x, cmd.angular.z);
    }
    else {
        // это сообщение будет печататься не чаще 1 раза в секунду
        ROS_INFO_STREAM_THROTTLE(1.0, "no control");
    }
    ROS_DEBUG_STREAM("cmd v = "<<cmd.linear.x<<" "<<cmd.angular.z);
    // отправляем (публикуем) команду
    cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  /**
   * Инициализация системы сообщений ros
   * Регистрация node с определенным именем (третий аргумент функции)
   * Эта функция должна быть вызвана в первую очередь
   */
  ros::init(argc, argv, "control_node");

  /**
   * NodeHandle  - объект через который осуществляется взаимодействие с ROS:
   * передача сообщений
   * регистрация коллбаков (функций обработки сообщений)
   */
  ros::NodeHandle node("~");

  /**
   * subscribe() функция подписки на сообщения определенного типа,
   * передаваемое по заданному топику
   * В качестве параметров указываются
   * - топик - на сообщения которого происходит подписка
   * - длина очереди сообщений хранящихся до обработки (если очередь заполняется,
   *  то самые старые сообщения будут автоматически удаляться )
   *  - функция обработки сообщений
   *
   *
   *  Подписываемся на данные дальномера

   */
  ros::Subscriber laser_sub = node.subscribe("/scan", 1, laserCallback);

  /*
   * Подписываемся на данные о положении робота
   */
  ros::Subscriber pose_sub = node.subscribe("/base_pose_ground_truth", 1, poseCallback);


  /*
   * Подписываемся на управление переключением алгоритмов
   */
  ros::Subscriber select_sub = node.subscribe("/selector", 100, selectCallback);

  /*
   * Регистрируем функцию обработчик таймера 10Hz
   */
  ros::Timer timer1 = node.createTimer(ros::Duration(0.1), timerCallback);

  /*
   * Сообщаем, что мы будем публиковать сообщения типа Twist по топику cmd_vel
   * второй параметр - длина очереди
   */
  cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

   //инициализация массива доступных алгоритмов
  controls[DUMMY] = new DummyControl();
  controls[VOYAGER] = new VoyagerControl(node.param("min_range", 1.0),
                                         node.param("max_vel", 0.5),
                                         node.param("max_omega", 0.5));
  // здесь должны быть инициализированы и другие алгоритмы
  // controls[WALLFOLLOWER] = new WallFollower(...)


  //устанавливаем указатель на действующий алгоритм управления на любой (например первый) элемент массива
  controlPtr = controls[VOYAGER];

   /**
   * ros::spin() функция внутри которой происходит вся работа по приему сообщений
   * и вызову соответствующих обработчиков . Вся обработка происходит из основного потока
   * (того, который вызвал ros::spin(), то есть основного в данном случае)
   * Функция будет завершена, когда подьзователь прервет выполнение процесса с Ctrl-C
   *
   */
  ros::spin();


  // сюда мы не попадем, но ради приличия в конце работы удалим динамически выделенную памать
  for (std::size_t i = 0; i < nControls; ++i)
  {
      delete controls[i];
  }

  return 0;
}
