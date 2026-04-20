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

#include "frame_utils.hpp"

// Forward declarations: Tells the compiler these classes exist without loading
// the whole file.
namespace Ogre {
class Camera;
class Rectangle2D;
class RenderTexture;
class SceneManager;
class SceneNode;
class Viewport;
} // namespace Ogre

namespace as2_camera_overlay {

/**
 * @brief The "Artist" class. Handles the Ogre3D graphics engine.
 *
 * This class is responsible for the actual rendering logic:
 * 1. Managing the 3D scene (adding lights, cameras).
 * 2. Handling the "Background Quad" (where the real video is displayed).
 * 3. Managing the "Render Target" (a hidden buffer in memory to draw into).
 * 4. Converting raw pixels between ROS, Ogre, and OpenCV.
 */
class OverlayRenderer {
public:
  OverlayRenderer();
  ~OverlayRenderer();

  // Prevent copying this class.
  OverlayRenderer(const OverlayRenderer &) = delete;
  OverlayRenderer &operator=(const OverlayRenderer &) = delete;

  /**
   * @brief Connects to Ogre and starts the 3D scene.
   */
  void initialize();

  /**
   * @brief Sets the color of the empty sky if no background is provided.
   */
  void setBackgroundColor(const Ogre::ColourValue &color);

  /**
   * @brief Toggles whether the raw video is displayed behind the 3D markers.
   */
  void setShowCameraBackground(bool show);

  /**
   * @brief Creates a memory buffer of a specific size to draw into.
   */
  void ensureRenderTarget(unsigned int width, unsigned int height);

  /**
   * @brief Configures the virtual camera to match a real camera's focal length.
   */
  void setIntrinsics(const Intrinsics &k, float near_plane, float far_plane,
                     float zoom_factor = 1.0f);

  /**
   * @brief Moves the virtual camera to a 3D position in the scene.
   */
  void setCameraPose(const Ogre::Vector3 &position,
                     const Ogre::Quaternion &orientation);

  /**
   * @brief Pastes a ROS Image into the background of the 3D scene.
   */
  void updateBackgroundImage(const sensor_msgs::msg::Image &image);

  /**
   * @brief Pastes an OpenCV image into the background of the 3D scene.
   */
  void updateBackgroundImage(const cv::Mat &bgr_or_bgra);

  /**
   * @brief Command Ogre to render the scene and read the pixels back into an
   * OpenCV image.
   * @return Finished image (with 3D markers drawn on top of the video).
   */
  cv::Mat renderAndRead();

  // Accessors for internal Ogre objects.
  Ogre::SceneManager *sceneManager() const { return scene_manager_; }
  Ogre::SceneNode *rootNode() const { return root_node_; }

private:
  void createSceneResources();
  void createBackgroundQuad();
  void destroyBackgroundTexture();
  void destroyRenderTarget();

  // --- Ogre Objects ---
  Ogre::SceneManager *scene_manager_{nullptr}; ///< The "World" manager.
  Ogre::SceneNode *root_node_{nullptr}; ///< The center (0,0,0) of the world.
  Ogre::Camera *camera_{nullptr};       ///< The virtual lens.
  Ogre::SceneNode *camera_node_{
      nullptr}; ///< The physical position of the lens.
  Ogre::SceneNode *background_node_{
      nullptr}; ///< Container for the background rectangle.
  Ogre::Rectangle2D *background_rect_{
      nullptr}; ///< The flat panel behind the 3D world.
  Ogre::MaterialPtr background_material_; ///< The "paint" applied to the panel.
  Ogre::TexturePtr
      background_texture_; ///< The actual video pixels as a texture.
  bool show_background_{false};

  // --- Drawing Buffer (The hidden screen) ---
  Ogre::TexturePtr render_texture_;
  Ogre::RenderTexture *render_target_{nullptr};
  Ogre::Viewport *viewport_{nullptr};
  unsigned int render_width_{0};
  unsigned int render_height_{0};
  Ogre::ColourValue clear_color_{0.0f, 0.0f, 0.0f, 1.0f};

  // Cache to avoid recalculating projection matrix if nothing changed.
  Intrinsics cached_intrinsics_;
  float cached_near_{0.0f};
  float cached_far_{0.0f};
  float cached_zoom_{0.0f};

  std::string
      unique_suffix_; ///< Used to name Ogre resources so they don't collide.
};

} // namespace as2_camera_overlay

#endif // AS2_CAMERA_OVERLAY__OVERLAY_RENDERER_HPP_
