#include "as2_camera_overlay/overlay_display_base.hpp"

#include <OgreSceneNode.h>

namespace as2_camera_overlay
{

void OverlayDisplayBase::setEnabled(bool enabled)
{
  enabled_ = enabled;
}

}  // namespace as2_camera_overlay
