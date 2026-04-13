#include "as2_camera_overlay/overlay_renderer.hpp"

#include <atomic>
#include <stdexcept>
#include <vector>

#include "as2_camera_overlay/frame_helpers.hpp"

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
#include <OgreHardwarePixelBuffer.h>
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
  background_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
  background_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

  Ogre::AxisAlignedBox aab_inf;
  aab_inf.setInfinite();
  background_rect_->setBoundingBox(aab_inf);

  const std::string material_name = "overlay_background_mat" + unique_suffix_;
  background_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto * technique = background_material_->getTechnique(0);
  auto * pass = technique->getPass(0);
  pass->setLightingEnabled(false);
  pass->setDepthCheckEnabled(false);
  pass->setDepthWriteEnabled(false);
  pass->setCullingMode(Ogre::CULL_NONE);
  pass->createTextureUnitState();
  background_material_->load();
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
    bg_texture_width_ = 0;
    bg_texture_height_ = 0;
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
    Ogre::PF_R8G8B8,
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

void OverlayRenderer::setIntrinsics(const Intrinsics & k, float near_plane, float far_plane)
{
  if (!k.valid()) {
    return;
  }
  ensureRenderTarget(k.width, k.height);
  camera_->setNearClipDistance(near_plane);
  camera_->setFarClipDistance(far_plane);
  const Ogre::Matrix4 proj = buildProjectionMatrix(k, near_plane, far_plane);
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
  if (image.width == 0 || image.height == 0) {
    return;
  }

  if (!background_texture_ ||
    bg_texture_width_ != image.width ||
    bg_texture_height_ != image.height)
  {
    destroyBackgroundTexture();
    const std::string name = "overlay_bg_texture" + unique_suffix_;
    background_texture_ = Ogre::TextureManager::getSingleton().createManual(
      name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      image.width,
      image.height,
      0,
      Ogre::PF_BYTE_RGB,
      Ogre::TU_DYNAMIC_WRITE_ONLY);
    bg_texture_width_ = image.width;
    bg_texture_height_ = image.height;

    auto * tu = background_material_->getTechnique(0)->getPass(0)->getTextureUnitState(0);
    tu->setTexture(background_texture_);
  }

  auto pixel_buffer = background_texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox & dest_box = pixel_buffer->getCurrentLock();
  auto * dest = static_cast<uint8_t *>(dest_box.data);

  const bool is_rgb = image.encoding == "rgb8" || image.encoding == "rgb";
  const bool is_bgr = image.encoding == "bgr8" || image.encoding == "bgr";
  const size_t row_bytes = image.width * 3;

  for (size_t y = 0; y < image.height; ++y) {
    const uint8_t * src = image.data.data() + y * image.step;
    uint8_t * dst = dest + y * dest_box.rowPitch * 3;
    if (is_rgb) {
      std::memcpy(dst, src, row_bytes);
    } else if (is_bgr) {
      for (size_t x = 0; x < image.width; ++x) {
        dst[x * 3 + 0] = src[x * 3 + 2];
        dst[x * 3 + 1] = src[x * 3 + 1];
        dst[x * 3 + 2] = src[x * 3 + 0];
      }
    } else {
      std::memcpy(dst, src, std::min(row_bytes, static_cast<size_t>(image.step)));
    }
  }

  pixel_buffer->unlock();
}

cv::Mat OverlayRenderer::renderAndRead()
{
  if (render_target_ == nullptr) {
    return cv::Mat();
  }
  scene_manager_->_updateSceneGraph(camera_);
  render_target_->update();

  cv::Mat out(render_height_, render_width_, CV_8UC3);
  Ogre::PixelBox pb(
    render_width_, render_height_, 1, Ogre::PF_BYTE_BGR, out.data);
  render_target_->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);
  return out;
}

}  // namespace as2_camera_overlay
