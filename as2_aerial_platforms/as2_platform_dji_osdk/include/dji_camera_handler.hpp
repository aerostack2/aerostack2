#ifndef DJI_CAMERA_HANDLER_HPP_
#define DJI_CAMERA_HANDLER_HPP_

// dji includes
#include "as2_core/node.hpp"
// #include "dji_liveview.hpp"
#include "dji_vehicle.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "osdk_platform.h"
#include "osdkhal_linux.h"

#include "dji_advanced_sensing.hpp"
#include "dji_linux_helpers.hpp"

using namespace DJI::OSDK;

void camera_cb(CameraRGBImage pImg, void* userData);
void main_camera_cb(CameraRGBImage pImg, void* userData);

class DJICameraHandler {
  DJI::OSDK::Vehicle* vehicle_ptr_;
  as2::Node* node_ptr_;

 public:
  DJICameraHandler(DJI::OSDK::Vehicle* vehicle, as2::Node* node)
      : vehicle_ptr_(vehicle), node_ptr_(node){};

  void start_camera() {
    if (vehicle_ptr_->getFwVersion() < Version::M100_31) {
      std::cout << "Camera is not supported on this platform" << std::endl;
      return;
    }

    // vehicle_ptr_->advancedSensing->startMainCameraStream(camera_cb,nullptr);
    /* if(vehicle_ptr_->advancedSensing->startFPVCameraStream(camera_cb,nullptr)){
      std::cout << "FPV camera stream started" << std::endl;
    }
    else{
      std::cout << "FPV camera stream failed to start" << std::endl;
    } */
    if (vehicle_ptr_->advancedSensing->startMainCameraStream(main_camera_cb,
                                                             nullptr)) {
      std::cout << "Main camera stream started" << std::endl;
    } else {
      std::cout << "Main camera stream failed to start" << std::endl;
    }

    // AdvancedSensing::startH264Stream(LiveView::LiveViewCameraPosition pos,
    // H264Callback cb, void *userData);
  }
  void stop_camera() {
    vehicle_ptr_->advancedSensing->stopFPVCameraStream();
    vehicle_ptr_->advancedSensing->stopMainCameraStream();
  }
};

#endif  // DJI_CAMERA_HANDLER_HPP_
