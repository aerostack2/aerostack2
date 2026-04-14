#ifndef AS2_CAMERA_OVERLAY__DISPLAYS__MARKER_ARRAY_DISPLAY_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAYS__MARKER_ARRAY_DISPLAY_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include <OgreMaterial.h>

#include "as2_camera_overlay/overlay_display_base.hpp"

namespace Ogre
{
class Entity;
class SceneManager;
class SceneNode;
}
namespace rviz_rendering
{
class Arrow;
class BillboardLine;
class MovableText;
class Shape;
}

namespace as2_camera_overlay
{

class MarkerArrayDisplay : public OverlayDisplayBase
{
public:
  MarkerArrayDisplay();
  ~MarkerArrayDisplay() override;

  void onInitialize(const DisplayContext & context) override;
  void update(const rclcpp::Time & stamp, const std::string & fixed_frame) override;
  void setEnabled(bool enabled) override;

private:
  using MarkerMsg = visualization_msgs::msg::Marker;
  using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

  struct MarkerKey
  {
    std::string ns;
    int32_t id;
    bool operator==(const MarkerKey & o) const {return id == o.id && ns == o.ns;}
  };
  struct MarkerKeyHash
  {
    size_t operator()(const MarkerKey & k) const noexcept
    {
      return std::hash<std::string>{}(k.ns) ^ (std::hash<int32_t>{}(k.id) << 1);
    }
  };

  struct MarkerNode
  {
    Ogre::SceneNode * scene_node{nullptr};
    std::vector<std::unique_ptr<rviz_rendering::Shape>> shapes;
    std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows;
    std::vector<std::unique_ptr<rviz_rendering::MovableText>> texts;
    std::vector<std::unique_ptr<rviz_rendering::BillboardLine>> lines;
    Ogre::Entity * entity{nullptr};
    std::set<Ogre::MaterialPtr> materials;
    std::string mesh_resource;
    bool mesh_use_embedded_materials{false};
    int32_t last_marker_type{-1};
    size_t last_point_count{0};
  };

  void topicCallback(MarkerArrayMsg::ConstSharedPtr msg);
  void processPendingMarkers(const std::string & fixed_frame);
  void applyMarker(const MarkerMsg & marker, const std::string & fixed_frame);
  void eraseMarker(const MarkerKey & key);
  void clearAll();

  bool computeWorldPose(
    const MarkerMsg & marker, const std::string & fixed_frame,
    Ogre::Vector3 & world_pos, Ogre::Quaternion & world_rot);

  void buildShape(
    const MarkerMsg & marker, MarkerNode & node,
    const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot);
  void buildArrow(
    const MarkerMsg & marker, MarkerNode & node,
    const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot);
  void buildLineList(
    const MarkerMsg & marker, MarkerNode & node,
    const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot,
    bool strip);
  void buildPoints(
    const MarkerMsg & marker, MarkerNode & node,
    const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot);
  void buildList(
    const MarkerMsg & marker, MarkerNode & node,
    const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot);
  void buildText(
    const MarkerMsg & marker, MarkerNode & node,
    const Ogre::Vector3 & world_pos);
  void buildMesh(
    const MarkerMsg & marker, MarkerNode & node,
    const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot);
  void destroyMarkerEntity(MarkerNode & node);

  Ogre::SceneNode * root_node_{nullptr};
  Ogre::SceneManager * scene_manager_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unordered_map<MarkerKey, std::unique_ptr<MarkerNode>, MarkerKeyHash> markers_;

  std::vector<rclcpp::Subscription<MarkerArrayMsg>::SharedPtr> subs_;
  std::mutex mutex_;
  std::deque<MarkerArrayMsg::ConstSharedPtr> pending_;

  rclcpp::Logger logger_{rclcpp::get_logger("as2_camera_overlay.MarkerArrayDisplay")};
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__DISPLAYS__MARKER_ARRAY_DISPLAY_HPP_
