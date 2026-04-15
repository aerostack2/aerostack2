#ifndef AS2_CAMERA_OVERLAY__OVERLAY_RENDERER_HPP_
#define AS2_CAMERA_OVERLAY__OVERLAY_RENDERER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreTexture.h>
#include <OgreVector.h>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera_projection.hpp"

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

#endif // AS2_CAMERA_OVERLAY__OVERLAY_RENDERER_HPP_
