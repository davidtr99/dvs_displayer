#include "dvs_displayer/displayer.h"
#include "dvs_displayer/custom_cmaps.h"

namespace dvs_displayer
{

  Displayer::Displayer(ros::NodeHandle &nh, ros::NodeHandle nh_private) : _nh(nh)
  {

    // subscribers & publishers
    _event_sub = _nh.subscribe("events", 1, &Displayer::eventsCallback, this);

    image_transport::ImageTransport it_(_nh);
    _viz_image_pub = it_.advertise("event_image", 1);

    // dynamic reconfigure
    _dynamic_reconfigure_cb = boost::bind(&Displayer::reconfigureCallback, this, _1, _2);
    _dynamic_reconfigure_server.reset(new dynamic_reconfigure::Server<dvs_displayer::dvs_displayerConfig>(nh_private));
    _dynamic_reconfigure_server->setCallback(_dynamic_reconfigure_cb);
  }

  Displayer::~Displayer()
  {
    _viz_image_pub.shutdown();
  }

  void Displayer::evaluateParameters(dvs_displayer::dvs_displayerConfig &config)
  {
    std::string color_map_str;
    color_map_str = config.color_map;
    
    if (color_map_str == std::string("grayscale"))
    {
      _color_map_config = ColorMap::GRAYSCALE;
    }
    else if (color_map_str == std::string("seismic"))
    {
      _color_map_config = ColorMap::SEISMIC_BIPOLAR;
    }
    else if (color_map_str == std::string("viridis"))
    {
      _color_map_config = ColorMap::VIRIDIS_UNIPOLAR;
    }
    else
    {
      ROS_WARN("Unknown color map '%s'. Using 'seismic' instead.", color_map_str.c_str());
      _color_map_config = ColorMap::SEISMIC_BIPOLAR;
    }
    std::string display_method_str;
    display_method_str = config.display_method;

    if (display_method_str == std::string("histogram"))
    {
      _display_method_config = DisplayMethod::HISTOGRAM;
    }
    else if (display_method_str == std::string("ternary"))
    {
      _display_method_config = DisplayMethod::TERNARY;
    }
    else if (display_method_str == std::string("time_surface"))
    {
      _display_method_config = DisplayMethod::TIME_SURFACE;
    }
    else
    {
      ROS_WARN("Unknown display method '%s'. Using 'histogram' instead.", display_method_str.c_str());
      _display_method_config = DisplayMethod::HISTOGRAM;
    }
  }

  void Displayer::add_color_cmap(cv::Mat &lut)
  {
    if (_color_map_config == ColorMap::VIRIDIS_UNIPOLAR)
    {
      dvs_displayer::viridis_cmap(lut);
    }
    else
    {
      dvs_displayer::seismic_cmap(lut);
    }
  }

  void Displayer::eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
  {

    if (_viz_image_pub.getNumSubscribers() <= 0) return;
    if (msg->events.size() <= 0) return;

    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = msg->events[msg->events.size() / 2].ts;

    
    switch (_display_method_config)
    {
      case DisplayMethod::HISTOGRAM:
        displayHistogram(cv_image, msg);
        break;
      case DisplayMethod::TERNARY:
        displayTernary(cv_image, msg);
        break;
      case DisplayMethod::TIME_SURFACE:
        displayTimeSurface(cv_image, msg);
        break;
    }
    _viz_image_pub.publish(cv_image.toImageMsg());
  }

  void Displayer::displayHistogram(cv_bridge::CvImage &cv_image, const dvs_msgs::EventArray::ConstPtr &msg)
  {
    // on-off event histograms
    cv::Mat on_events = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    cv::Mat off_events = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    for (const dvs_msgs::Event &ev : msg->events)
    {
      if (ev.polarity == true)
        on_events.at<uint8_t>(ev.y, ev.x)++;
      else
        off_events.at<uint8_t>(ev.y, ev.x)++;
    }

    // unipolar image 
    if (_color_map_config == ColorMap::VIRIDIS_UNIPOLAR)
    {
      cv::Mat gray_image = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(0));
      cv::Mat events = cv::Mat::zeros(msg->height, msg->width, CV_8U);
      cv::add(on_events, off_events, events);

      // scale image to use the full range [0,255]
      double max_events, dummy;
      cv::minMaxLoc(events, &dummy, &max_events);
      const double scale = 255 / (max_events*0.1);
      gray_image = min(scale * events, 255);

      cv::Mat cm_img;
      cv::Mat gray_image3ch;
      cv::cvtColor(gray_image, gray_image3ch, CV_GRAY2BGR);
      cv::LUT(gray_image3ch, _custom_color_map, cm_img);
      cv_image.encoding = "bgr8";
      cv_image.image = cm_img;
      return;
    }

    // bipolar image
    cv::Mat gray_image = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));
    double max_on, max_off, dummy;
    cv::minMaxLoc(on_events, &dummy, &max_on);
    cv::minMaxLoc(off_events, &dummy, &max_off);
    const double max_abs_val = std::max(max_on, max_off);
    const double scale = 127 / max_abs_val;
    gray_image += scale * on_events;
    gray_image -= scale * off_events;

    // use a colormap
    if (_color_map_config == ColorMap::SEISMIC_BIPOLAR)
    {
      cv::Mat cm_img;
      cv::Mat gray_image3ch;
      cv::cvtColor(gray_image, gray_image3ch, CV_GRAY2BGR);
      cv::LUT(gray_image3ch, _custom_color_map, cm_img);
      cv_image.encoding = "bgr8";
      cv_image.image = cm_img;
    }
    else if (_color_map_config == ColorMap::GRAYSCALE)
    {
      cv_image.encoding = "mono8";
      cv_image.image = gray_image;
    }
  }

  void Displayer::displayTernary(cv_bridge::CvImage &cv_image, const dvs_msgs::EventArray::ConstPtr &msg)
  {
    // on-off event maps
    cv::Mat on_events = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    cv::Mat off_events = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    for (const dvs_msgs::Event &ev : msg->events)
    {
      if (ev.polarity == true)
        on_events.at<uint8_t>(ev.y, ev.x) = 1;
      else
        off_events.at<uint8_t>(ev.y, ev.x) = 1; 
    }

    // ternary image
    cv::Mat ternary_image = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));
    ternary_image.setTo(255, on_events);
    ternary_image.setTo(0, off_events);

    // use a colormap
    if (_color_map_config != ColorMap::GRAYSCALE)
    {
      cv::Mat cm_img;
      cv::Mat gray_image3ch;
      cv::cvtColor(ternary_image, gray_image3ch, CV_GRAY2BGR);
      cv::LUT(gray_image3ch, _custom_color_map, cm_img);
      cv_image.encoding = "bgr8";
      cv_image.image = cm_img;
    }
    else
    {
      cv_image.encoding = "mono8";
      cv_image.image = ternary_image;
    }
  }

  void Displayer::displayTimeSurface(cv_bridge::CvImage &cv_image, const dvs_msgs::EventArray::ConstPtr &msg)
  {
    const double TAU{0.01};
    cv::Mat ts_events = cv::Mat::zeros(msg->height, msg->width, CV_8U);

    double last_time = msg->events[0].ts.toSec();
    for (const dvs_msgs::Event &ev : msg->events)
    {
      ts_events.at<uint8_t>(ev.y, ev.x) = 255 * exp(-abs(last_time - ev.ts.toSec())/TAU);
    }

    // use a colormap
    if (_color_map_config != ColorMap::GRAYSCALE)
    {
      cv::Mat cm_img;
      cv::Mat gray_image3ch;
      cv::cvtColor(ts_events, gray_image3ch, CV_GRAY2BGR);
      cv::LUT(gray_image3ch, _custom_color_map, cm_img);
      cv_image.encoding = "bgr8";
      cv_image.image = cm_img;
    }
    else
    {
      cv_image.encoding = "mono8";
      cv_image.image = ts_events;
    }
  }

  void Displayer::reconfigureCallback(dvs_displayer::dvs_displayerConfig &config, uint32_t level)
  {
    evaluateParameters(config);
    if (_color_map_config != ColorMap::GRAYSCALE)
    {
      _custom_color_map = cv::Mat(1, 256, CV_8UC3);
      add_color_cmap(_custom_color_map);
    }
  }

} // namespace
