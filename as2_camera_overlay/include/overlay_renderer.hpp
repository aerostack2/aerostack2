// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!******************************************************************************
 *  \file       overlay_renderer.hpp
 *  \brief      overlay renderer implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#ifndef AS2_CAMERA_OVERLAY__OVERLAY_RENDERER_HPP_
#define AS2_CAMERA_OVERLAY__OVERLAY_RENDERER_HPP_
#include "frame_utils.hpp"
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreTexture.h>
#include <OgreVector.h>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
namespace Ogre {
class Camera;
class Rectangle2D;
class RenderTexture;
class SceneManager;
class SceneNode;
class Viewport;
} // namespace Ogre
namespace as2_camera_overlay {
class OverlayRenderer {
public:
  OverlayRenderer();
  ~OverlayRenderer();
  OverlayRenderer(const OverlayRenderer &) = delete;
  OverlayRenderer &operator=(const OverlayRenderer &) = delete;
  void initialize();
  void setBackgroundColor(const Ogre::ColourValue &color);
  void setShowCameraBackground(bool show);
  void ensureRenderTarget(unsigned int width, unsigned int height);
  void setIntrinsics(const Intrinsics &k, float near_plane, float far_plane,
                     float zoom_factor = 1.0f);
  void setCameraPose(const Ogre::Vector3 &position,
                     const Ogre::Quaternion &orientation);
  void updateBackgroundImage(const sensor_msgs::msg::Image &image);
  void updateBackgroundImage(const cv::Mat &bgr_or_bgra);
  cv::Mat renderAndRead();
  Ogre::SceneManager *sceneManager() const { return scene_manager_; }
  Ogre::SceneNode *rootNode() const { return root_node_; }

private:
  void createSceneResources();
  void createBackgroundQuad();
  void destroyBackgroundTexture();
  void destroyRenderTarget();
  Ogre::SceneManager *scene_manager_{nullptr};
  Ogre::SceneNode *root_node_{nullptr};
  Ogre::Camera *camera_{nullptr};
  Ogre::SceneNode *camera_node_{nullptr};
  Ogre::SceneNode *background_node_{nullptr};
  Ogre::Rectangle2D *background_rect_{nullptr};
  Ogre::MaterialPtr background_material_;
  Ogre::TexturePtr background_texture_;
  bool show_background_{false};
  Ogre::TexturePtr render_texture_;
  Ogre::RenderTexture *render_target_{nullptr};
  Ogre::Viewport *viewport_{nullptr};
  unsigned int render_width_{0};
  unsigned int render_height_{0};
  Ogre::ColourValue clear_color_{0.0f, 0.0f, 0.0f, 1.0f};
  Intrinsics cached_intrinsics_;
  float cached_near_{0.0f};
  float cached_far_{0.0f};
  float cached_zoom_{0.0f};
  std::string unique_suffix_;
};
} // namespace as2_camera_overlay
#endif
