#include "displays/marker_array_display.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <OgreColourValue.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>
#include <OgreVector.h>
#include <OgreQuaternion.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/mesh_loader.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include "frame_helpers.hpp"
#include "param_helpers.hpp"

namespace as2_camera_overlay
{

namespace
{
Ogre::ColourValue toColour(const std_msgs::msg::ColorRGBA & c)
{
  return Ogre::ColourValue(c.r, c.g, c.b, c.a);
}
}  // namespace

MarkerArrayDisplay::MarkerArrayDisplay() = default;
MarkerArrayDisplay::~MarkerArrayDisplay()
{
  for (auto & kv : markers_) {
    if (kv.second) {
      destroyMarkerEntity(*kv.second);
    }
  }
}

void MarkerArrayDisplay::onInitialize(const DisplayContext & context)
{
  name_ = context.display_name;
  tf_buffer_ = context.tf_buffer;
  scene_manager_ = context.scene_manager;
  logger_ = context.node->get_logger().get_child("MarkerArrayDisplay");

  const std::string ns = context.param_namespace + ".";
  auto * node = context.node;

  const std::vector<std::string> topics = getOrDeclare<std::vector<std::string>>(
    node, ns + "topics",
    std::vector<std::string>{"gates_static", "gates_tracking"});
  const int queue_size = getOrDeclare<int>(node, ns + "queue_size", 100);

  root_node_ = context.root_node->createChildSceneNode();

  for (const auto & topic : topics) {
    auto cb = [this](MarkerArrayMsg::ConstSharedPtr msg) {this->topicCallback(msg);};
    auto sub = node->create_subscription<MarkerArrayMsg>(topic, queue_size, cb);
    subs_.push_back(sub);
    RCLCPP_INFO(logger_, "Subscribed to MarkerArray topic '%s'", topic.c_str());
  }
}

void MarkerArrayDisplay::topicCallback(MarkerArrayMsg::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  pending_.push_back(std::move(msg));
}

void MarkerArrayDisplay::update(
  const rclcpp::Time &, const std::string & fixed_frame)
{
  processPendingMarkers(fixed_frame);
}

void MarkerArrayDisplay::processPendingMarkers(const std::string & fixed_frame)
{
  std::deque<MarkerArrayMsg::ConstSharedPtr> local;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    local.swap(pending_);
  }
  for (const auto & msg : local) {
    for (const auto & m : msg->markers) {
      switch (m.action) {
        case MarkerMsg::ADD:
          applyMarker(m, fixed_frame);
          break;
        case MarkerMsg::DELETE:
          eraseMarker({m.ns, m.id});
          break;
        case MarkerMsg::DELETEALL:
          clearAll();
          break;
        default:
          applyMarker(m, fixed_frame);
          break;
      }
    }
  }
  for (auto & kv : markers_) {
    if (kv.second && kv.second->scene_node != nullptr) {
      kv.second->scene_node->setVisible(isEnabled());
    }
  }
}

void MarkerArrayDisplay::eraseMarker(const MarkerKey & key)
{
  auto it = markers_.find(key);
  if (it == markers_.end()) {return;}
  if (it->second) {
    destroyMarkerEntity(*it->second);
    if (it->second->scene_node != nullptr) {
      scene_manager_->destroySceneNode(it->second->scene_node);
    }
  }
  markers_.erase(it);
}

void MarkerArrayDisplay::clearAll()
{
  for (auto & kv : markers_) {
    if (kv.second) {
      destroyMarkerEntity(*kv.second);
      if (kv.second->scene_node != nullptr) {
        scene_manager_->destroySceneNode(kv.second->scene_node);
      }
    }
  }
  markers_.clear();
}

bool MarkerArrayDisplay::computeWorldPose(
  const MarkerMsg & marker, const std::string & fixed_frame,
  Ogre::Vector3 & world_pos, Ogre::Quaternion & world_rot)
{
  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_rot;
  std::string err;
  if (!lookupTransformOgre(
      *tf_buffer_, fixed_frame, marker.header.frame_id, rclcpp::Time(0, 0),
      frame_pos, frame_rot, &err))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 5000,
      "MarkerArrayDisplay: TF lookup %s -> %s failed: %s",
      fixed_frame.c_str(), marker.header.frame_id.c_str(), err.c_str());
    return false;
  }
  Ogre::Vector3 local_pos;
  Ogre::Quaternion local_rot;
  poseToOgre(marker.pose, local_pos, local_rot);
  world_pos = frame_pos + frame_rot * local_pos;
  world_rot = frame_rot * local_rot;
  return true;
}

void MarkerArrayDisplay::applyMarker(
  const MarkerMsg & marker, const std::string & fixed_frame)
{
  const MarkerKey key{marker.ns, marker.id};

  Ogre::Vector3 world_pos;
  Ogre::Quaternion world_rot;
  if (!computeWorldPose(marker, fixed_frame, world_pos, world_rot)) {
    return;
  }

  auto it = markers_.find(key);
  if (it == markers_.end()) {
    auto mn = std::make_unique<MarkerNode>();
    mn->scene_node = root_node_->createChildSceneNode();
    it = markers_.emplace(key, std::move(mn)).first;
  }
  auto & mn = *it->second;

  const size_t new_point_count = marker.points.size();
  const bool type_changed = (mn.last_marker_type != marker.type);
  const bool points_changed = (mn.last_point_count != new_point_count);

  // In-place update for simple shape types when type and point count haven't changed
  if (!type_changed && !points_changed) {
    switch (marker.type) {
      case MarkerMsg::CUBE:
      case MarkerMsg::SPHERE:
      case MarkerMsg::CYLINDER:
        if (mn.shapes.size() == 1) {
          mn.shapes[0]->setPosition(world_pos);
          mn.shapes[0]->setOrientation(world_rot);
          mn.shapes[0]->setScale(Ogre::Vector3(
            static_cast<float>(marker.scale.x),
            static_cast<float>(marker.scale.y),
            static_cast<float>(marker.scale.z)));
          mn.shapes[0]->setColor(toColour(marker.color));
          mn.scene_node->setVisible(isEnabled());
          return;
        }
        break;
      case MarkerMsg::ARROW:
        if (mn.arrows.size() == 1) {
          mn.arrows[0]->setPosition(world_pos);
          mn.arrows[0]->setOrientation(
            world_rot * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
          mn.arrows[0]->setColor(toColour(marker.color));
          mn.scene_node->setVisible(isEnabled());
          return;
        }
        break;
      case MarkerMsg::MESH_RESOURCE:
        buildMesh(marker, mn, world_pos, world_rot);
        mn.scene_node->setVisible(isEnabled());
        return;
      default:
        break;
    }
  }

  // Full rebuild needed
  destroyMarkerEntity(mn);
  if (mn.scene_node != nullptr && !mn.texts.empty()) {
    mn.scene_node->detachAllObjects();
  }

  mn.shapes.clear();
  mn.arrows.clear();
  mn.lines.clear();
  mn.texts.clear();
  mn.last_marker_type = marker.type;
  mn.last_point_count = new_point_count;

  switch (marker.type) {
    case MarkerMsg::CUBE:
    case MarkerMsg::SPHERE:
    case MarkerMsg::CYLINDER:
      buildShape(marker, mn, world_pos, world_rot);
      break;
    case MarkerMsg::ARROW:
      buildArrow(marker, mn, world_pos, world_rot);
      break;
    case MarkerMsg::LINE_STRIP:
      buildLineList(marker, mn, world_pos, world_rot, true);
      break;
    case MarkerMsg::LINE_LIST:
      buildLineList(marker, mn, world_pos, world_rot, false);
      break;
    case MarkerMsg::POINTS:
      buildPoints(marker, mn, world_pos, world_rot);
      break;
    case MarkerMsg::CUBE_LIST:
    case MarkerMsg::SPHERE_LIST:
      buildList(marker, mn, world_pos, world_rot);
      break;
    case MarkerMsg::MESH_RESOURCE:
      buildMesh(marker, mn, world_pos, world_rot);
      break;
    case MarkerMsg::TEXT_VIEW_FACING:
      buildText(marker, mn, world_pos);
      break;
    default:
      RCLCPP_WARN_THROTTLE(
        logger_, *rclcpp::Clock::make_shared(), 10000,
        "MarkerArrayDisplay: marker type %d not supported yet (ns=%s id=%d)",
        marker.type, marker.ns.c_str(), marker.id);
      break;
  }
  mn.scene_node->setVisible(isEnabled());
}

void MarkerArrayDisplay::buildShape(
  const MarkerMsg & marker, MarkerNode & node,
  const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot)
{
  rviz_rendering::Shape::Type type = rviz_rendering::Shape::Cube;
  switch (marker.type) {
    case MarkerMsg::SPHERE: type = rviz_rendering::Shape::Sphere; break;
    case MarkerMsg::CYLINDER: type = rviz_rendering::Shape::Cylinder; break;
    case MarkerMsg::CUBE:
    case MarkerMsg::MESH_RESOURCE:
    default: type = rviz_rendering::Shape::Cube; break;
  }
  auto shape = std::make_unique<rviz_rendering::Shape>(type, scene_manager_, node.scene_node);
  shape->setPosition(world_pos);
  shape->setOrientation(world_rot);
  shape->setScale(
    Ogre::Vector3(
      static_cast<float>(marker.scale.x),
      static_cast<float>(marker.scale.y),
      static_cast<float>(marker.scale.z)));
  shape->setColor(toColour(marker.color));
  node.shapes.push_back(std::move(shape));
}

void MarkerArrayDisplay::buildArrow(
  const MarkerMsg & marker, MarkerNode & node,
  const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot)
{
  auto arrow = std::make_unique<rviz_rendering::Arrow>(
    scene_manager_, node.scene_node,
    static_cast<float>(marker.scale.x * 0.8), static_cast<float>(marker.scale.y),
    static_cast<float>(marker.scale.x * 0.2), static_cast<float>(marker.scale.z));
  arrow->setColor(toColour(marker.color));
  arrow->setPosition(world_pos);
  arrow->setOrientation(
    world_rot * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
  node.arrows.push_back(std::move(arrow));
}

void MarkerArrayDisplay::buildLineList(
  const MarkerMsg & marker, MarkerNode & node,
  const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot,
  bool strip)
{
  if (marker.points.empty()) {return;}
  auto line = std::make_unique<rviz_rendering::BillboardLine>(
    scene_manager_, node.scene_node);
  line->setLineWidth(static_cast<float>(std::max(marker.scale.x, 0.01)));
  line->setColor(marker.color.r, marker.color.g, marker.color.b, marker.color.a);

  if (strip) {
    line->setMaxPointsPerLine(static_cast<uint32_t>(marker.points.size()));
    line->setNumLines(1);
    for (const auto & p : marker.points) {
      const Ogre::Vector3 local = toOgreVector(p);
      line->addPoint(world_pos + world_rot * local);
    }
  } else {
    const size_t pairs = marker.points.size() / 2;
    line->setMaxPointsPerLine(2);
    line->setNumLines(static_cast<uint32_t>(pairs));
    for (size_t i = 0; i < pairs; ++i) {
      const Ogre::Vector3 a = world_pos + world_rot * toOgreVector(marker.points[2 * i]);
      const Ogre::Vector3 b = world_pos + world_rot * toOgreVector(marker.points[2 * i + 1]);
      line->addPoint(a);
      line->addPoint(b);
      line->finishLine();
    }
  }
  node.lines.push_back(std::move(line));
}

void MarkerArrayDisplay::buildPoints(
  const MarkerMsg & marker, MarkerNode & node,
  const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot)
{
  const float size = std::max(static_cast<float>(marker.scale.x), 0.02f);
  for (size_t i = 0; i < marker.points.size(); ++i) {
    auto shape = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, scene_manager_, node.scene_node);
    const Ogre::Vector3 local = toOgreVector(marker.points[i]);
    shape->setPosition(world_pos + world_rot * local);
    shape->setScale(Ogre::Vector3(size, size, size));
    if (i < marker.colors.size()) {
      shape->setColor(toColour(marker.colors[i]));
    } else {
      shape->setColor(toColour(marker.color));
    }
    node.shapes.push_back(std::move(shape));
  }
}

void MarkerArrayDisplay::buildList(
  const MarkerMsg & marker, MarkerNode & node,
  const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot)
{
  const auto type = (marker.type == MarkerMsg::SPHERE_LIST) ?
    rviz_rendering::Shape::Sphere : rviz_rendering::Shape::Cube;
  for (size_t i = 0; i < marker.points.size(); ++i) {
    auto shape = std::make_unique<rviz_rendering::Shape>(type, scene_manager_, node.scene_node);
    const Ogre::Vector3 local = toOgreVector(marker.points[i]);
    shape->setPosition(world_pos + world_rot * local);
    shape->setOrientation(world_rot);
    shape->setScale(
      Ogre::Vector3(
        static_cast<float>(marker.scale.x),
        static_cast<float>(marker.scale.y),
        static_cast<float>(marker.scale.z)));
    if (i < marker.colors.size()) {
      shape->setColor(toColour(marker.colors[i]));
    } else {
      shape->setColor(toColour(marker.color));
    }
    node.shapes.push_back(std::move(shape));
  }
}
void MarkerArrayDisplay::destroyMarkerEntity(MarkerNode & node)
{
  if (node.entity != nullptr) {
    if (node.scene_node != nullptr) {
      node.scene_node->detachObject(node.entity);
    }
    scene_manager_->destroyEntity(node.entity);
    node.entity = nullptr;
  }
  for (auto & mat : node.materials) {
    if (mat) {
      mat->unload();
      Ogre::MaterialManager::getSingletonPtr()->remove(mat->getName(), mat->getGroup());
    }
  }
  node.materials.clear();
  node.mesh_resource.clear();
}

void MarkerArrayDisplay::buildMesh(
  const MarkerMsg & marker, MarkerNode & node,
  const Ogre::Vector3 & world_pos, const Ogre::Quaternion & world_rot)
{
  if (marker.mesh_resource.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 10000,
      "MarkerArrayDisplay: MESH_RESOURCE marker has empty mesh_resource (ns=%s id=%d)",
      marker.ns.c_str(), marker.id);
    return;
  }

  if (node.entity != nullptr &&
    node.mesh_resource == marker.mesh_resource &&
    node.mesh_use_embedded_materials == marker.mesh_use_embedded_materials)
  {
    node.scene_node->setPosition(world_pos);
    node.scene_node->setOrientation(world_rot);
    node.scene_node->setScale(
      Ogre::Vector3(
        static_cast<float>(marker.scale.x),
        static_cast<float>(marker.scale.y),
        static_cast<float>(marker.scale.z)));
    return;
  }

  Ogre::MeshPtr mesh = rviz_rendering::loadMeshFromResource(marker.mesh_resource);
  if (!mesh) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 10000,
      "MarkerArrayDisplay: failed to load mesh '%s' (ns=%s id=%d)",
      marker.mesh_resource.c_str(), marker.ns.c_str(), marker.id);
    return;
  }

  static uint32_t mesh_counter = 0;
  std::string entity_id = "overlay_mesh_" + std::to_string(mesh_counter++);
  node.entity = scene_manager_->createEntity(entity_id, marker.mesh_resource);
  node.scene_node->attachObject(node.entity);
  node.mesh_resource = marker.mesh_resource;
  node.mesh_use_embedded_materials = marker.mesh_use_embedded_materials;

  Ogre::MaterialPtr default_mat =
    rviz_rendering::MaterialManager::createMaterialWithLighting(entity_id + "Material");
  default_mat->getTechnique(0)->setAmbient(0.5f, 0.5f, 0.5f);
  node.materials.insert(default_mat);

  if (marker.mesh_use_embedded_materials) {
    for (uint32_t i = 0; i < node.entity->getNumSubEntities(); ++i) {
      std::string mat_name = node.entity->getSubEntity(i)->getMaterialName();
      if (mat_name != "BaseWhiteNoLighting") {
        auto orig = Ogre::MaterialManager::getSingleton().getByName(
          mat_name, node.entity->getSubEntity(i)->getMaterial()->getGroup());
        if (orig) {
          Ogre::MaterialPtr clone = orig->clone(entity_id + mat_name);
          node.entity->getSubEntity(i)->setMaterialName(clone->getName());
          node.materials.insert(clone);
        }
      } else {
        node.entity->getSubEntity(i)->setMaterial(default_mat);
      }
    }
  } else {
    node.entity->setMaterial(default_mat);
  }

  const float r = marker.color.r;
  const float g = marker.color.g;
  const float b = marker.color.b;
  const float a = marker.color.a;
  for (auto & mat : node.materials) {
    Ogre::Technique * tech = mat->getTechnique(0);
    Ogre::Pass * pass = tech->getPass(0);
    if (!marker.mesh_use_embedded_materials) {
      pass->setAmbient(r * 0.5f, g * 0.5f, b * 0.5f);
      pass->setDiffuse(r, g, b, a);
    }
    Ogre::SceneBlendType blending;
    bool depth_write;
    rviz_rendering::MaterialManager::enableAlphaBlending(blending, depth_write, a);
    pass->setSceneBlending(blending);
    pass->setDepthWriteEnabled(depth_write);
    pass->setLightingEnabled(true);
  }

  node.scene_node->setPosition(world_pos);
  node.scene_node->setOrientation(world_rot);
  node.scene_node->setScale(
    Ogre::Vector3(
      static_cast<float>(marker.scale.x),
      static_cast<float>(marker.scale.y),
      static_cast<float>(marker.scale.z)));
}

void MarkerArrayDisplay::buildText(
  const MarkerMsg & marker, MarkerNode & node,
  const Ogre::Vector3 & world_pos)                                                                                              
{                                                                                                                               
  if (marker.text.empty()) {return;}                                                                                            
  const float height = std::max(static_cast<float>(marker.scale.z), 0.05f);                                                     
  auto text = std::make_unique<rviz_rendering::MovableText>(                                                                    
    marker.text, "Liberation Sans", height, toColour(marker.color));                                                            
  text->setTextAlignment(                                                                                                       
    rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);                                              
  node.scene_node->setPosition(world_pos);                                                                                      
  node.scene_node->attachObject(text.get());                                                                                    
  node.texts.push_back(std::move(text));                                                                                        
}   
void MarkerArrayDisplay::setEnabled(bool enabled)
{
  OverlayDisplayBase::setEnabled(enabled);
  if (root_node_ != nullptr) {
    root_node_->setVisible(enabled);
  }
}

}  // namespace as2_camera_overlay

PLUGINLIB_EXPORT_CLASS(
  as2_camera_overlay::MarkerArrayDisplay, as2_camera_overlay::OverlayDisplayBase)
