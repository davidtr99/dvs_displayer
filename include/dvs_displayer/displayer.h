#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_displayer/dvs_displayerConfig.h>

namespace dvs_displayer
{
class Displayer 
{
  enum DisplayMethod
  {
    HISTOGRAM, TERNARY, TIME_SURFACE
  };
  enum ColorMap
  {
    GRAYSCALE, SEISMIC_BIPOLAR, VIRIDIS_UNIPOLAR
  };

public:
  Displayer(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Displayer();

private:
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void add_color_cmap(cv::Mat& lut);
  void reconfigureCallback(dvs_displayer::dvs_displayerConfig &config, uint32_t level);
  void evaluateParameters(dvs_displayer::dvs_displayerConfig &config);

  void displayHistogram(cv_bridge::CvImage &cv_image, const dvs_msgs::EventArray::ConstPtr &msg);
  void displayTernary(cv_bridge::CvImage &cv_image, const dvs_msgs::EventArray::ConstPtr &msg);
  void displayTimeSurface(cv_bridge::CvImage &cv_image, const dvs_msgs::EventArray::ConstPtr &msg);

private:
  ros::NodeHandle _nh;

  ros::Subscriber _event_sub;
  image_transport::Publisher _viz_image_pub;

  DisplayMethod _display_method_config;
  ColorMap _color_map_config;

  cv::Mat _custom_color_map;

  boost::shared_ptr<dynamic_reconfigure::Server<dvs_displayer::dvs_displayerConfig> > _dynamic_reconfigure_server;
  dynamic_reconfigure::Server<dvs_displayer::dvs_displayerConfig>::CallbackType _dynamic_reconfigure_cb;
};

} // namespace
