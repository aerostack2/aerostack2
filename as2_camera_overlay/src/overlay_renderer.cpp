#include "overlay_renderer.hpp"

#include <atomic>
#include <stdexcept>
#include <vector>

#include "frame_helpers.hpp"

#include <rcutils/logging_macros.h>

#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/render_system.hpp>

// X11 headers (pulled via Ogre) leak macros that collide with rclcpp enums
#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef Status
#undef Status
#endif
#ifdef Always
#undef Always
#endif

#include <OgreCamera.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <OgreHardwarePixelBuffer.h>
#include <OgreImage.h>
#include <OgreMaterialManager.h>
#include <OgrePixelFormat.h>
#include <OgreRectangle2D.h>
#include <OgreRenderTexture.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

namespace as2_camera_overlay
{

namespace
{
std::atomic<uint32_t> g_renderer_counter{0};
std::atomic_bool g_warned_empty_background{false};
std::atomic_bool g_warned_conversion_failure{false};
}  // namespace

OverlayRenderer::OverlayRenderer()
{
  unique_suffix_ = "_" + std::to_string(g_renderer_counter.fetch_add(1));
}

OverlayRenderer::~OverlayRenderer()
{
  destroyRenderTarget();
  destroyBackgroundTexture();
  if (scene_manager_ != nullptr) {
    auto * root = Ogre::Root::getSingletonPtr();
    if (root != nullptr) {
      root->destroySceneManager(scene_manager_);
    }
    scene_manager_ = nullptr;
  }
}

void OverlayRenderer::initialize()
{
  auto * render_system = rviz_rendering::RenderSystem::get();
  if (render_system == nullptr) {
    throw std::runtime_error("rviz_rendering::RenderSystem::get() returned null");
  }
  auto * ogre_root = render_system->getOgreRoot();
  if (ogre_root == nullptr) {
    throw std::runtime_error("Ogre::Root is null — RenderSystem not initialized");
  }

  scene_manager_ = ogre_root->createSceneManager(
    Ogre::ST_GENERIC, "as2_camera_overlay_scene" + unique_suffix_);
  scene_manager_->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
  render_system->prepareOverlays(scene_manager_);

  root_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(
    "overlay_root" + unique_suffix_);

  camera_ = scene_manager_->createCamera("overlay_camera" + unique_suffix_);
  camera_->setNearClipDistance(0.01f);
  camera_->setFarClipDistance(1000.0f);
  camera_->setFixedYawAxis(false);
  camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(
    "overlay_camera_node" + unique_suffix_);
  camera_node_->attachObject(camera_);

  auto * light_node = scene_manager_->getRootSceneNode()->createChildSceneNode(
    "overlay_light_node" + unique_suffix_);
  auto * light = scene_manager_->createLight("overlay_light" + unique_suffix_);
  light->setType(Ogre::Light::LT_DIRECTIONAL);
  light->setDirection(Ogre::Vector3(-0.5f, -0.5f, -1.0f).normalisedCopy());
  light->setDiffuseColour(1.0f, 1.0f, 1.0f);
  light->setSpecularColour(1.0f, 1.0f, 1.0f);
  light_node->attachObject(light);

  createSceneResources();
}

void OverlayRenderer::createSceneResources()
{
  createBackgroundQuad();
}

void OverlayRenderer::createBackgroundQuad()
{
  background_rect_ = new Ogre::Rectangle2D(true);
  background_rect_->setUseIdentityProjection(true);
  background_rect_->setUseIdentityView(true);
  background_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
  background_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

  Ogre::AxisAlignedBox aab_inf;
  aab_inf.setInfinite();
  background_rect_->setBoundingBox(aab_inf);

  const std::string material_name = "overlay_background_mat" + unique_suffix_;
  background_material_ =
    rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
  background_material_->setDepthWriteEnabled(false);
  background_material_->setDepthCheckEnabled(false);
  background_material_->setCullingMode(Ogre::CULL_NONE);
  background_material_->setSceneBlending(Ogre::SBT_REPLACE);

  auto * tu = background_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  background_rect_->setMaterial(background_material_);

  background_node_ = root_node_->createChildSceneNode("overlay_background_node" + unique_suffix_);
  background_node_->attachObject(background_rect_);
  background_node_->setVisible(false);
}

void OverlayRenderer::destroyBackgroundTexture()
{
  if (background_texture_) {
    Ogre::TextureManager::getSingleton().remove(background_texture_->getHandle());
    background_texture_.reset();
  }
}

void OverlayRenderer::destroyRenderTarget()
{
  if (render_texture_) {
    Ogre::TextureManager::getSingleton().remove(render_texture_->getHandle());
    render_texture_.reset();
    render_target_ = nullptr;
    viewport_ = nullptr;
    render_width_ = 0;
    render_height_ = 0;
  }
}

void OverlayRenderer::setBackgroundColor(const Ogre::ColourValue & color)
{
  clear_color_ = color;
  if (viewport_ != nullptr) {
    viewport_->setBackgroundColour(clear_color_);
  }
}

void OverlayRenderer::setShowCameraBackground(bool show)
{
  show_background_ = show;
  if (background_node_ != nullptr) {
    background_node_->setVisible(show);
  }
}

void OverlayRenderer::ensureRenderTarget(unsigned int width, unsigned int height)
{
  if (width == 0 || height == 0) {
    return;
  }
  if (render_texture_ && render_width_ == width && render_height_ == height) {
    return;
  }
  destroyRenderTarget();

  const std::string name = "overlay_rtt" + unique_suffix_;
  render_texture_ = Ogre::TextureManager::getSingleton().createManual(
    name,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D,
    width,
    height,
    0,
    Ogre::PF_BYTE_BGRA,
    Ogre::TU_RENDERTARGET);

  render_target_ = render_texture_->getBuffer()->getRenderTarget();
  render_target_->setAutoUpdated(false);

  viewport_ = render_target_->addViewport(camera_);
  viewport_->setClearEveryFrame(true);
  viewport_->setBackgroundColour(clear_color_);
  viewport_->setOverlaysEnabled(false);

  camera_->setAspectRatio(
    static_cast<Ogre::Real>(width) / static_cast<Ogre::Real>(height));

  render_width_ = width;
  render_height_ = height;
}

void OverlayRenderer::setIntrinsics(const Intrinsics & k, float near_plane, float far_plane, float zoom_factor)
{
  if (!k.valid()) {
    return;
  }
  if (k == cached_intrinsics_ && near_plane == cached_near_ &&
    far_plane == cached_far_ && zoom_factor == cached_zoom_)
  {
    return;
  }
  cached_intrinsics_ = k;
  cached_near_ = near_plane;
  cached_far_ = far_plane;
  cached_zoom_ = zoom_factor;

  camera_->setNearClipDistance(near_plane);
  camera_->setFarClipDistance(far_plane);
  const Ogre::Matrix4 proj = buildProjectionMatrix(k, near_plane, far_plane, zoom_factor);
  camera_->setCustomProjectionMatrix(true, proj);
}

void OverlayRenderer::setCameraPose(
  const Ogre::Vector3 & position, const Ogre::Quaternion & orientation)
{
  camera_node_->setPosition(position);
  camera_node_->setOrientation(orientation);
}

void OverlayRenderer::updateBackgroundImage(const sensor_msgs::msg::Image & image)
{
  if (image.width == 0 || image.height == 0 || image.data.empty()) {
    if (!g_warned_empty_background.exchange(true)) {
      RCUTILS_LOG_WARN_NAMED(
        "as2_camera_overlay.overlay_renderer",
        "Skipping background image update: empty image or data buffer.");
    }
    return;
  }

  Ogre::PixelFormat pf = Ogre::PF_UNKNOWN;
  const uint8_t * data_ptr = image.data.data();
  cv_bridge::CvImagePtr converted_img;
  const auto & enc = image.encoding;

  if (enc == sensor_msgs::image_encodings::RGB8) {
    pf = Ogre::PF_BYTE_RGB;
  } else if (enc == sensor_msgs::image_encodings::RGBA8) {
    pf = Ogre::PF_BYTE_RGBA;
  } else if (
    enc == sensor_msgs::image_encodings::TYPE_8UC4 ||
    enc == sensor_msgs::image_encodings::TYPE_8SC4 ||
    enc == sensor_msgs::image_encodings::BGRA8)
  {
    pf = Ogre::PF_BYTE_BGRA;
  } else if (
    enc == sensor_msgs::image_encodings::TYPE_8UC3 ||
    enc == sensor_msgs::image_encodings::TYPE_8SC3 ||
    enc == sensor_msgs::image_encodings::BGR8)
  {
    pf = Ogre::PF_BYTE_BGR;
  } else if (
    enc == sensor_msgs::image_encodings::TYPE_8UC1 ||
    enc == sensor_msgs::image_encodings::TYPE_8SC1 ||
    enc == sensor_msgs::image_encodings::MONO8 ||
    enc.rfind("bayer", 0) == 0)
  {
    pf = Ogre::PF_BYTE_L;
  } else {
    // Convert unsupported encodings to bgr8 via cv_bridge
    try {
      converted_img = cv_bridge::toCvCopy(image, "bgr8");
      pf = Ogre::PF_BYTE_BGR;
      data_ptr = converted_img->image.data;
    } catch (const cv_bridge::Exception &) {
      if (!g_warned_conversion_failure.exchange(true)) {
        RCUTILS_LOG_WARN_NAMED(
          "as2_camera_overlay.overlay_renderer",
          "Failed converting input image encoding '%s' to bgr8.", image.encoding.c_str());
      }
      return;
    }
  }

  if (background_texture_ && (background_texture_->getWidth() != image.width ||
                              background_texture_->getHeight() != image.height ||
                              background_texture_->getFormat() != pf)) {
    destroyBackgroundTexture();
  }

  Ogre::PixelBox pb(image.width, image.height, 1, pf, const_cast<uint8_t *>(data_ptr));

  if (!background_texture_) {
    const std::string tex_name = "overlay_bg_texture" + unique_suffix_;
    background_texture_ = Ogre::TextureManager::getSingleton().createManual(
      tex_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      image.width,
      image.height,
      0,
      pf,
      Ogre::TU_DEFAULT);

    background_texture_->getBuffer()->blitFromMemory(pb);
    auto * tu = background_material_->getTechnique(0)->getPass(0)->getTextureUnitState(0);
    tu->setTexture(background_texture_);
  } else {
    background_texture_->getBuffer()->blitFromMemory(pb);
  }
}

void OverlayRenderer::updateBackgroundImage(const cv::Mat & bgr_or_bgra)
{
  if (bgr_or_bgra.empty()) {
    return;
  }
  const unsigned int w = static_cast<unsigned int>(bgr_or_bgra.cols);
  const unsigned int h = static_cast<unsigned int>(bgr_or_bgra.rows);
  const Ogre::PixelFormat pf =
    (bgr_or_bgra.channels() == 4) ? Ogre::PF_BYTE_BGRA : Ogre::PF_BYTE_BGR;

  if (background_texture_ && (background_texture_->getWidth() != w ||
    background_texture_->getHeight() != h ||
    background_texture_->getFormat() != pf))
  {
    destroyBackgroundTexture();
  }

  Ogre::PixelBox pb(w, h, 1, pf, const_cast<unsigned char *>(bgr_or_bgra.data));

  if (!background_texture_) {
    const std::string tex_name = "overlay_bg_texture" + unique_suffix_;
    background_texture_ = Ogre::TextureManager::getSingleton().createManual(
      tex_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, w, h, 0, pf, Ogre::TU_DEFAULT);
    background_texture_->getBuffer()->blitFromMemory(pb);
    auto * tu = background_material_->getTechnique(0)->getPass(0)->getTextureUnitState(0);
    tu->setTexture(background_texture_);
  } else {
    background_texture_->getBuffer()->blitFromMemory(pb);
  }
}

cv::Mat OverlayRenderer::renderAndRead()
{
  if (render_target_ == nullptr) {
    return cv::Mat();
  }
  scene_manager_->_updateSceneGraph(camera_);
  render_target_->update();

  cv::Mat out(render_height_, render_width_, CV_8UC4);
  Ogre::PixelBox pb(
    render_width_, render_height_, 1, Ogre::PF_BYTE_BGRA, out.data);
  pb.rowPitch = static_cast<size_t>(out.step / out.elemSize());
  render_target_->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);
  return out;
}

}  // namespace as2_camera_overlay
