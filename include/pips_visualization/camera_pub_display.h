/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_CAMERA_PUB_DISPLAY_H
#define RVIZ_CAMERA_PUB_DISPLAY_H


#include <string>

#include "rviz/default_plugin/camera_display.h"

#include <cv_bridge/cv_bridge.h>

#ifndef Q_MOC_RUN
#include <OgreTexture.h>
#endif


namespace video_export
{
class  VideoPublisher;
}

namespace rviz
{

class EnumProperty;
class FloatProperty;
class IntProperty;
class RenderPanel;
class RosTopicProperty;
class DisplayGroupVisibilityProperty;
class ColorProperty;

/**
 * \class CameraPubDisplay
 *
 */
class CameraPubDisplay: public CameraDisplay
{
  Q_OBJECT
public:
  CameraPubDisplay();
  virtual ~CameraPubDisplay();

  // Overrides from Display
  virtual void onInitialize();
//  virtual void fixedFrameChanged();
//  virtual void update(float wall_dt, float ros_dt);
//  virtual void reset();

  // Overrides from Ogre::RenderTargetListener
//  virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
  virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

//  static const QString BACKGROUND;
//  static const QString OVERLAY;
//  static const QString BOTH;

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();
 // virtual void processMessage(const sensor_msgs::Image::ConstPtr& msg);

private Q_SLOTS:
//  void forceRender();
//  void updateAlpha();

  void updateTopic();
//  virtual void updateQueueSize();
//  virtual void updateFrameRate();
//  virtual void updateBackgroundColor();
//  virtual void updateDisplayNamespace();

private:
//  std::string camera_trigger_name_;
  ros::NodeHandle nh_;

  void advertise();
  void unadvertise();
  
  cv_bridge::CvImagePtr cv_ptr_;

//  ros::ServiceServer trigger_service_;
//  bool triggerCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
//  bool trigger_activated_;
//  ros::Time last_image_publication_time_;

//  void caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

//  bool updateCamera();

//  void clear();
//  void updateStatus();

//  ros::Subscriber caminfo_sub_;

  RosTopicProperty* topic_property_;
//  RosTopicProperty* camera_info_property_;
//  DisplayGroupVisibilityProperty* visibility_property_;
//  IntProperty* queue_size_property_;
//  StringProperty* namespace_property_;

//  FloatProperty* frame_rate_property_;
//  ColorProperty* background_color_property_;

//  sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
//  boost::mutex caminfo_mutex_;

//  bool new_caminfo_;

//  bool caminfo_ok_;

//  bool force_render_;

//  uint32_t vis_bit_;

  video_export::VideoPublisher* video_publisher_;

  // render to texture
  // from http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Intermediate+Tutorial+7
//  Ogre::Camera* camera_;
//  Ogre::TexturePtr rtt_texture_;
//  Ogre::RenderTexture* render_texture_;
};

}  // namespace rviz

#endif  // RVIZ_CAMERA_PUB_DISPLAY_H
