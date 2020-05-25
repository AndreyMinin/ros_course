/*
 * lane_detector.cpp
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try
  {
    //конвертируем сообщение в объект CvMat OpenCv
    img = cv_bridge::toCvShare(msg, "bgr8")->image;

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  //указываем степень обработки
#define PHASE 0

#if PHASE >= 0
  //0. конвертируем в оттенки серого
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
#endif

#if PHASE == 0
  //конвертируем изоражение в сообщение
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_gray).toImageMsg();
#endif

#if PHASE >= 1
  //1. размытие изображения
  //контейнер для оработанного изображения
  cv::Mat img_blur;
  //операция размытия 3ий параметр - размер ядра сглаживания
  cv::GaussianBlur(img_gray, img_blur, cv::Size(5, 5), 1);
#endif

#if PHASE == 1
  //конвертируем изоражение в сообщение
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_blur).toImageMsg();
#endif

#if PHASE >= 2
  //2. выделение краев  по минимальному и максимальному порогу
  cv::Mat img_canny;
  int min_threshold = 200;
  int max_threshold = 255;
  cv::Canny(img_blur, img_canny, min_threshold, max_threshold );
#endif

#if PHASE == 2
  //конвертируем изоражение в сообщение
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_canny).toImageMsg();
#endif

#if PHASE >= 3
  //3. Маскирование региона интереса
  //создаем картинку, равную по размеру исходной, и заполняем её нулями
  cv::Mat roi_mask = cv::Mat::zeros(img_canny.rows, img_canny.cols, img_canny.type());
  //Координаты полигона
  cv::Point poly[1][4];
  poly[0][0] = cv::Point(0,0);
  poly[0][1] = cv::Point(800, 20);
  poly[0][2] = cv::Point(800, 500);
  poly[0][3] = cv::Point(0, 500);
  const cv::Point* ppt[1] = { poly[0] };
  int npt[] = { 4 };
  int line_type = cv::LINE_8;
  int pixel_mask = 0xFF; //255 - маска на каждый пиксель
  int poly_number = 1;
  cv::fillPoly(roi_mask, ppt, npt, poly_number, pixel_mask, line_type);
  //маскируем картинку - все , что не попало в полигон - будет черным (0)
  cv::Mat masked_canny;
  cv::bitwise_and(img_canny, roi_mask, masked_canny);
#endif

#if PHASE == 3
  //конвертируем в сообщение
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", masked_canny).toImageMsg();
#endif


#if PHASE >= 4
  //4. Применяем преобразование Хафа
  //контейнер для линий
  std::vector<cv::Vec4i> lines;
  //разрешение по расстоянию
  double rho = 1.0;
  //разрешение по углу
  double theta = 0.3;
  //порог голосов за линию
  double threshold = 10;
  //минимальная длина линий
  double min_len = 50;
  //максимальный разрыв
  double max_gap = 10;

  cv::HoughLinesP(masked_canny, lines, rho,theta, threshold, min_len, max_gap  );

  //рисуем линии на исходном изображении
  for( size_t i = 0; i < lines.size(); i++ )
  {
      cv::line(img, cv::Point(lines[i][0], lines[i][1]),
          cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
  }
#endif

#if PHASE == 4
  //конвертируем изображение в сообщение
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
#endif
  //отправляем
  pub.publish(out_msg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
  pub = it.advertise("processed_image", 10);
  ros::spin();

}


