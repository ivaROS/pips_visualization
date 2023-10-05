#include <pips_visualization/camera_pub_display.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <OgreRenderWindow.h>

namespace video_export
{
class VideoPublisher
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  uint image_id_;
public:
  VideoPublisher() :
    it_(nh_),
    image_id_(0)
  {
  }
  
    image_transport::Publisher pub_;

  std::string get_topic()
  {
    return pub_.getTopic();
  }

  bool is_active()
  {
    return !pub_.getTopic().empty();
  }

  void setNodehandle(const ros::NodeHandle& nh)
  {
    shutdown();
    nh_ = nh;
    it_ = image_transport::ImageTransport(nh_);
  }

  void shutdown()
  {
    if (pub_.getTopic() != "")
    {
      pub_.shutdown();
    }
  }

  void advertise(std::string topic)
  {
    pub_ = it_.advertise(topic, 1);
  }



  // bool publishFrame(Ogre::RenderWindow * render_object, const std::string frame_id)
  bool publishFrame(Ogre::RenderTexture * render_object, const std::string frame_id)
  {
    if (pub_.getTopic() == "")
    {
      return false;
    }
    if (frame_id == "")
    {
      return false;
    }
    // RenderTarget::writeContentsToFile() used as example
    int height = render_object->getHeight();
    int width = render_object->getWidth();
    // the results of pixel format have to be used to determine
    // image.encoding
    Ogre::PixelFormat pf = render_object->suggestPixelFormat();
    uint pixelsize = Ogre::PixelUtil::getNumElemBytes(pf);
    uint datasize = width * height * pixelsize;

    // 1.05 multiplier is to avoid crash when the window is resized.
    // There should be a better solution.
    uchar *data = OGRE_ALLOC_T(uchar, datasize * 1.05, Ogre::MEMCATEGORY_RENDERSYS);
    Ogre::PixelBox pb(width, height, 1, pf, data);
    render_object->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);

    sensor_msgs::Image image;
    image.header.stamp = ros::Time::now();
    image.header.seq = image_id_++;
    image.header.frame_id = frame_id;
    image.height = height;
    image.width = width;
    image.step = pixelsize * width;
    if (pixelsize == 3)
      image.encoding = sensor_msgs::image_encodings::RGB8;  // would break if pf changes
    else if (pixelsize == 4)
      image.encoding = sensor_msgs::image_encodings::RGBA8;  // would break if pf changes
    else
    {
      ROS_ERROR_STREAM("unknown pixel format " << pixelsize << " " << pf);
    }
    image.is_bigendian = (OGRE_ENDIAN == OGRE_ENDIAN_BIG);
    image.data.resize(datasize);
    memcpy(&image.data[0], data, datasize);
    pub_.publish(image);

    OGRE_FREE(data, Ogre::MEMCATEGORY_RENDERSYS);
  }
  
  bool publishFrame(Ogre::RenderWindow * render_object, const std::string frame_id)
  {
    if (pub_.getTopic() == "")
    {
      return false;
    }
    if (frame_id == "")
    {
      return false;
    }
    // RenderTarget::writeContentsToFile() used as example
    int height = render_object->getHeight();
    int width = render_object->getWidth();
    // the results of pixel format have to be used to determine
    // image.encoding
    Ogre::PixelFormat pf = render_object->suggestPixelFormat();
    uint pixelsize = Ogre::PixelUtil::getNumElemBytes(pf);
    uint datasize = width * height * pixelsize;

    // 1.05 multiplier is to avoid crash when the window is resized.
    // There should be a better solution.
    uchar *data = OGRE_ALLOC_T(uchar, datasize * 1.05, Ogre::MEMCATEGORY_RENDERSYS);
    Ogre::PixelBox pb(width, height, 1, pf, data);
    render_object->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);

    sensor_msgs::Image image;
    image.header.stamp = ros::Time::now();
    image.header.seq = image_id_++;
    image.header.frame_id = frame_id;
    image.height = height;
    image.width = width;
    image.step = pixelsize * width;
    if (pixelsize == 3)
      image.encoding = sensor_msgs::image_encodings::RGB8;  // would break if pf changes
    else if (pixelsize == 4)
      image.encoding = sensor_msgs::image_encodings::RGBA8;  // would break if pf changes
    else
    {
      ROS_ERROR_STREAM("unknown pixel format " << pixelsize << " " << pf);
    }
    image.is_bigendian = (OGRE_ENDIAN == OGRE_ENDIAN_BIG);
    image.data.resize(datasize);
    memcpy(&image.data[0], data, datasize);
    pub_.publish(image);

    OGRE_FREE(data, Ogre::MEMCATEGORY_RENDERSYS);
  }
  
  /*
  bool publishFrame(cv::Mat& mat, const std::string frame_id)
  {
    if (pub_.getTopic() == "")
    {
      return false;
    }
    if (frame_id == "")
    {
      return false;
    }

    sensor_msgs::Image image;
    image.header.stamp = ros::Time::now();
    image.header.seq = image_id_++;
    image.header.frame_id = frame_id;
    image.height = height;
    image.width = width;
    image.step = pixelsize * width;
    if (pixelsize == 3)
      image.encoding = sensor_msgs::image_encodings::RGB8;  // would break if pf changes
    else if (pixelsize == 4)
      image.encoding = sensor_msgs::image_encodings::RGBA8;  // would break if pf changes
    else
    {
      ROS_ERROR_STREAM("unknown pixel format " << pixelsize << " " << pf);
    }
    image.is_bigendian = (OGRE_ENDIAN == OGRE_ENDIAN_BIG);
    image.data.resize(datasize);
    memcpy(&image.data[0], data, datasize);
    pub_.publish(image);

    OGRE_FREE(data, Ogre::MEMCATEGORY_RENDERSYS);
  }
  */
};
}  // namespace video_export



namespace rviz
{

CameraPubDisplay::CameraPubDisplay() : CameraDisplay()
{
  topic_property_ = new RosTopicProperty("Published Image Topic", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "sensor_msgs::Image topic to publish to.", this, SLOT(updateTopic()));

}

CameraPubDisplay::~CameraPubDisplay()
{
  delete video_publisher_;
}

void CameraPubDisplay::onInitialize()
{
  CameraDisplay::onInitialize();

  video_publisher_ = new video_export::VideoPublisher();
  
  
}

void CameraPubDisplay::updateTopic()
{
  unadvertise();
//  reset();
  advertise();
  context_->queueRender();
}


void CameraPubDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  CameraDisplay::postRenderTargetUpdate(evt);
  
  /*
  // Publish the rendered window video stream
  const ros::Time cur_time = ros::Time::now();
  ros::Duration elapsed_duration = cur_time - last_image_publication_time_;
  const float frame_rate = frame_rate_property_->getFloat();
  bool time_is_up = (frame_rate > 0.0) && (elapsed_duration.toSec() > 1.0 / frame_rate);
  // We want frame rate to be unlimited if we enter zero or negative values for frame rate
  if (frame_rate < 0.0)
  {
    time_is_up = true;
  }
  if (!(trigger_activated_ || time_is_up))
  {
    return;
  }
  trigger_activated_ = false;
  last_image_publication_time_ = cur_time;
  render_texture_->getViewport(0)->setBackgroundColour(background_color_property_->getOgreColor());

  */

  std::string frame_id;
  {
    //boost::mutex::scoped_lock lock(caminfo_mutex_);
    sensor_msgs::Image::ConstPtr image = texture_.getImage();
    if (!image)
      return;
    frame_id = image->header.frame_id;
  }

  // render_texture_->update();
  
  Ogre::TexturePtr texture_ptr_ = texture_.getTexture();
  
  //Ogre::RenderTexture* rtt_texture_ = texture_ptr_->getBuffer()->getRenderTarget();
  
  cv_ptr_ = cv_bridge::toCvCopy(texture_.getImage());
  
  QPixmap screenshot
      //  = QPixmap::grabWindow(context_->getViewManager()->getRenderPanel()->winId());
       = QPixmap::grabWindow(render_panel_->winId());
      QImage src = screenshot.toImage().convertToFormat(QImage::Format_RGB888);  // RGB
      cv::Mat image(src.height(), src.width(), CV_8UC3,
                    (uchar*)src.bits(), src.bytesPerLine());  // RGB
  
  
  ROS_INFO_STREAM_THROTTLE(1,"grabbing image... w=" << image.cols << ",h=" << image.rows);
  
  cv_ptr_->image = image;
  const sensor_msgs::Image::ConstPtr& msg = cv_ptr_->toImageMsg();
  
  //video_publisher_->publishFrame(rtt_texture_, frame_id);
  //    video_publisher_->pub_.publish(msg);
      
  Ogre::RenderWindow* window = render_panel_->getRenderWindow();
  video_publisher_->publishFrame(window, frame_id);
}
/*

void CameraPubDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  CameraDisplay::processMessage(msg);
  cv_ptr_ = cv_bridge::toCvShare(msg);
}
*/

void CameraPubDisplay::onEnable()
{
  CameraDisplay::onEnable();
  
  advertise();

}

void CameraPubDisplay::onDisable()
{
  CameraDisplay::onDisable();
  
  unadvertise();
  //clear();
}

void CameraPubDisplay::advertise()
{
  if (!isEnabled())
    return;

  std::string topic_name = topic_property_->getTopicStd();
  if (topic_name.empty())
  {
    setStatus(StatusProperty::Error, "Output Topic", "No topic set");
    return;
  }

  std::string error;
  if (!ros::names::validate(topic_name, error))
  {
    setStatus(StatusProperty::Error, "Output Topic", QString(error.c_str()));
    return;
  }


  video_publisher_->advertise(topic_name);
  setStatus(StatusProperty::Ok, "Output Topic", "Topic set");

}

void CameraPubDisplay::unadvertise()
{
  video_publisher_->shutdown();
}

//void CameraPubDisplay::update(float wall_dt, float ros_dt)

//bool CameraDisplay::updateCamera()

/*
void CameraPubDisplay::reset()
{
  CameraDisplay::reset();
 // clear();
}
*/

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::CameraPubDisplay, rviz::Display )
