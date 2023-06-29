// "Copyight [year] <Copyright Owner>"

#ifndef ___DJI_MATRICE_PLATFORM_HPP_
#define ___DJI_MATRICE_PLATFORM_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <string>

// ros includes
#include "as2_core/aerial_platform.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "dji_telemetry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

// dji includes
#include "dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include "osdk_platform.h"
#include "osdkhal_linux.h"

// opencv includes
#include <opencv2/opencv.hpp>
#include <thread>
#include <tuple>

#include "dji_camera_handler.hpp"
#include "dji_subscriber.hpp"
#include "opencv2/highgui/highgui.hpp"

#define RELIABLE_RECV_ONCE_BUFFER_SIZE (1024)
#define RELIABLE_SEND_ONCE_BUFFER_SIZE (1024)

bool getBroadcastData(DJI::OSDK::Vehicle *vehicle, int responseTimeout = 1);

class DJIMatricePlatform : public as2::AerialPlatform {
  bool enable_mop_channel_ = false;
  bool enable_advanced_sensing_ = false;
  bool has_mode_settled_ = false;
  bool command_changes_ = false;
  uint8_t control_flag_ = 0x00;
  DJI::OSDK::FlightController::JoystickMode dji_joystick_mode_;
  std::shared_ptr<LinuxSetup> linux_env_ptr_;
  Vehicle *vehicle_ = nullptr;

  bool publish_camera_ = false;

 public:
  DJIMatricePlatform(int argc, char **argv);
  ~DJIMatricePlatform() {
    for (auto &sub : dji_subscriptions_) {
      sub->stop();
    }
    delete vehicle_;
  };

  std::shared_ptr<DJICameraHandler> camera_handler_;
  std::shared_ptr<DJIGimbalHandler> gimbal_handler_;
  std::shared_ptr<DJICameraTrigger> camera_trigger_;

  std::vector<DJISubscription::SharedPtr> dji_subscriptions_;

  void configureSensors() override;
  // void publishSensorData()override {};

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(
      const as2_msgs::msg::ControlMode &msg) override;
  bool ownSendCommand() override;

  bool ownTakeoff() override;
  bool ownLand() override;

  void ownStopPlatform() override {
    vehicle_->flightController->emergencyBrakeAction();
  };
  void ownKillSwitch() override {
    RCLCPP_ERROR(get_logger(),
                 "Kill switch activated for DJI Matrice. \n A DJI won't kill "
                 "switch use the Remote Controller to land the drone.");
  };

  void downlinkCallback(const std_msgs::msg::String::SharedPtr msg);

 private:
  void printDJIError(ErrorCode::ErrorCodeType error);
  int djiInitVehicle();
  void djiReadTelemetry(){};
  void djiReadBattery(){};
  void djiConfigureSensors() {
    vehicle_->djiBattery->subscribeBatteryWholeInfo(true);
  };

 public:
  int enableDjiMopServer() {
    if (!vehicle_->initMopServer()) {
      RCLCPP_ERROR(this->get_logger(), "Error creating mop server");
      return -1;
    }
    RCLCPP_INFO(this->get_logger(), "Mop server created");
    return 0;
  }

  DJI::OSDK::MopPipeline *pipeline = NULL;

  MopErrCode acceptMopClient() {
    DJI::OSDK::MOP::PipelineID id(49152);
    DJI::OSDK::MOP::PipelineType type = DJI::OSDK::MOP::PipelineType::RELIABLE;
    MopErrCode ret = vehicle_->mopServer->accept(id, type, pipeline);
    return ret;
  }

  std::string readData(const uint8_t *data, size_t len) {
    std::string result(reinterpret_cast<const char *>(data), len);

    return result;
  }

  void start() {
    if (djiInitVehicle() < 0) {
      // RCLCPP_ERROR(get_logger(), "DJI Matrice Platform: Failed to initialize
      // vehicle.");
      throw std::runtime_error(
          "DJI Matrice Platform: Failed to initialize vehicle.");
      return;
    }

    configureSensors();

    for (auto &sub : dji_subscriptions_) {
      sub->start();
    }

    if (enable_mop_channel_) {
      enableDjiMopServer();
      DJI::OSDK::MOP::MopErrCode ret = acceptMopClient();
      RCLCPP_WARN(this->get_logger(),
                  "Code when calling accept mop conexion: %i", ret);
    }
    uint8_t *recvBuf;
    recvBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_RECV_ONCE_BUFFER_SIZE);

    if (recvBuf == NULL) {
      RCLCPP_ERROR(this->get_logger(), "Osdk_malloc recvbuffer error");
    }

    MopPipeline::DataPackType readPack = {(uint8_t *)recvBuf,
                                          RELIABLE_RECV_ONCE_BUFFER_SIZE};

    uint8_t *sendBuf;
    sendBuf = (uint8_t *)OsdkOsal_Malloc(RELIABLE_SEND_ONCE_BUFFER_SIZE);

    if (sendBuf == NULL) {
      RCLCPP_ERROR(this->get_logger(), "Osdk_malloc sendbuffer error");
    }

    MopPipeline::DataPackType writePack = {(uint8_t *)sendBuf,
                                           RELIABLE_SEND_ONCE_BUFFER_SIZE};
    int counter = 0;

    while (true) {
      memset(sendBuf, counter++, RELIABLE_SEND_ONCE_BUFFER_SIZE);
      writePack.length = RELIABLE_SEND_ONCE_BUFFER_SIZE;
      DJI::OSDK::MOP::MopErrCode ret =
          pipeline->sendData(writePack, &writePack.length);

      // if (ret != DJI::OSDK::MOP::MopErrCode::MOP_TIMEOUT) {
      RCLCPP_WARN(this->get_logger(), "Code when calling send: %i", ret);
      // }

      std::string datasent = readData(writePack.data, writePack.length);

      if (writePack.length != RELIABLE_SEND_ONCE_BUFFER_SIZE) {
        std::cout << "Data sent: " << datasent << std::endl;
        RCLCPP_INFO(this->get_logger(), "data length: %d", writePack.length);
      }

      memset(recvBuf, 0, RELIABLE_RECV_ONCE_BUFFER_SIZE);
      readPack.length = RELIABLE_RECV_ONCE_BUFFER_SIZE;
      ret = pipeline->recvData(readPack, &readPack.length);

      if (ret != DJI::OSDK::MOP::MopErrCode::MOP_TIMEOUT) {
        RCLCPP_WARN(this->get_logger(), "Code when calling recv: %i", ret);
      }

      std::string dataReceived = readData(readPack.data, readPack.length);

      if (readPack.length != RELIABLE_RECV_ONCE_BUFFER_SIZE) {
        std::cout << "Data received: " << dataReceived << std::endl;
        RCLCPP_INFO(this->get_logger(), "data length: %d", readPack.length);
      }
    }

    // ownSetArmingState(true);
  };

  void run_test() {
    if (djiInitVehicle() < 0) {
      return;
    }
    std::cout << "Vehicle initialized, starting.\n";

    // bool enableSubscribeBatteryWholeInfo = true;
    // BatteryWholeInfo batteryWholeInfo;
    // SmartBatteryDynamicInfo firstBatteryDynamicInfo;
    // SmartBatteryDynamicInfo secondBatteryDynamicInfo;
    // const int waitTimeMs = 500;
    // while (rclcpp::ok()) {
    //   vehicle_->djiBattery->getBatteryWholeInfo(batteryWholeInfo);
    //   DSTATUS("(It's valid only for M210V2)batteryCapacityPercentage is
    //   %ld%\n",
    //           batteryWholeInfo.batteryCapacityPercentage);
    //   vehicle_->djiBattery->getSingleBatteryDynamicInfo(
    //       DJIBattery::RequestSmartBatteryIndex::FIRST_SMART_BATTERY,
    //       firstBatteryDynamicInfo);
    //   DSTATUS("battery index %d batteryCapacityPercent is %ld%\n",
    //           firstBatteryDynamicInfo.batteryIndex,
    //           firstBatteryDynamicInfo.batteryCapacityPercent);
    //   DSTATUS("battery index %d currentVoltage is %ldV\n",
    //   firstBatteryDynamicInfo.batteryIndex,
    //           firstBatteryDynamicInfo.currentVoltage / 1000);
    //   DSTATUS("battery index %d batteryTemperature is %ld\n",
    //   firstBatteryDynamicInfo.batteryIndex,
    //           firstBatteryDynamicInfo.batteryTemperature / 10);
    //   vehicle_->djiBattery->getSingleBatteryDynamicInfo(
    //       DJIBattery::RequestSmartBatteryIndex::SECOND_SMART_BATTERY,
    //       secondBatteryDynamicInfo);
    //   DSTATUS("battery index %d batteryCapacityPercent is %ld%\n",
    //           secondBatteryDynamicInfo.batteryIndex,
    //           secondBatteryDynamicInfo.batteryCapacityPercent);
    //   DSTATUS("battery index %d currentVoltage is %ldV\n",
    //   secondBatteryDynamicInfo.batteryIndex,
    //           secondBatteryDynamicInfo.currentVoltage / 1000);
    //   DSTATUS("battery index %d batteryTemperature is %ld\n",
    //   secondBatteryDynamicInfo.batteryIndex,
    //           secondBatteryDynamicInfo.batteryTemperature / 10);
    //   OsdkOsal_TaskSleepMs(waitTimeMs);
    // }
    // getBroadcastData(vehicle_);
    //
  };
};

#endif  // DJI_MATRICE_PLATFORM_HPP_
