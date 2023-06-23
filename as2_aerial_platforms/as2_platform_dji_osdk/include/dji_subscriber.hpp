#ifndef ___DJI_SUBSCRIBER_HPP___
#define ___DJI_SUBSCRIBER_HPP___

#include <as2_core/sensor.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/magnetic_field__struct.hpp>
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/gps_utils.hpp"
#include "dji_linux_helpers.hpp"
#include "dji_telemetry.hpp"
#include "dji_vehicle.hpp"
#include "osdk_platform.h"
#include "osdkhal_linux.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#define RESPONSE_TIMEOUT 1

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class DJISubscription {
  static int n_packages_;

  std::shared_ptr<rclcpp::TimerBase> update_timer_;
  int package_index_;
  int frequency_;
  bool enable_timestamp_ = false;
  bool started_ = false;
  std::string name_;

 public:
  DJISubscription(const std::string package_name, as2::Node *&node,
                  Vehicle *vehicle, int frequency,
                  bool enable_timestamp = false)
      : name_(package_name),
        node_(node),
        vehicle_(vehicle),
        frequency_(frequency),
        enable_timestamp_(enable_timestamp) {
    if (vehicle_ == nullptr) throw std::runtime_error("Vehicle is nullptr");
    n_packages_++;
    package_index_ = n_packages_;
  };
  using SharedPtr = std::shared_ptr<DJISubscription>;

 private:
 protected:
  virtual void onStart(){};
  virtual void initializeTopics() = 0;
  virtual void onUpdate() = 0;
  virtual void onStop(){};

 public:
  bool start() {
    initializeTopics();
    onStart();

    // TopicName test[] = {TOPIC_RTK_CONNECT_STATUS};

    // int test_size = sizeof(test) / sizeof(test[0]);
    // RCLCPP_INFO(node_->get_logger(), "test size: %d", test_size);
    // // print vehicle_ ptr address
    // RCLCPP_INFO(node_->get_logger(), "vehicle ptr address: %p", vehicle_);

    auto subscribeStatus = vehicle_->subscribe->verify(1);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      return false;
    }

    // RCLCPP_INFO(node_->get_logger(), "subscribe status: %d",
    // subscribeStatus);
    bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(
        getPackageIndex(), topics_.size(), topics_.data(), getEnableTimestamp(),
        getFrequency());

    // bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(
    //     getPackageIndex(), test_size, (), getPackageIndex(),
    //     getEnableTimestamp());

    if (!pkgStatus) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize package %d : %s ",
                   getPackageIndex(), getName().c_str());
      return pkgStatus;
    }

    auto status =
        vehicle_->subscribe->startPackage(getPackageIndex(), RESPONSE_TIMEOUT);
    if (status.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE) {
      RCLCPP_WARN(node_->get_logger(), "%s  is not available",
                  getName().c_str());
      return false;
    }
    if (ACK::getError(status) != ACK::SUCCESS) {
      ACK::getErrorCodeMessage(status, __func__);
      // Cleanup before return
      vehicle_->subscribe->removePackage(getPackageIndex(), RESPONSE_TIMEOUT);
      return false;
    };

    update_timer_ =
        node_->create_timer(std::chrono::duration<double>(1.0f / frequency_),
                            std::bind(&DJISubscription::update, this));

    started_ = true;
    return true;
  };

  void stop() {
    update_timer_->cancel();
    vehicle_->subscribe->removePackage(getPackageIndex(), RESPONSE_TIMEOUT);
    onStop();
  };

  void update() {
    if (!started_) {
      return;
    }
    onUpdate();
  };

  inline int getPackageIndex() const { return package_index_; };
  inline float getFrequency() const { return frequency_; };
  inline void setFrequency(const float frequency) { frequency_ = frequency; };
  inline bool getEnableTimestamp() const { return enable_timestamp_; };
  inline std::string getName() const { return name_; };

 protected:
  Vehicle *vehicle_ = nullptr;
  std::vector<TopicName> topics_;
  as2::Node *node_;
};

class DJISubscriptionRTK : public DJISubscription {
 private:
  as2::sensors::GPS rtk_;
  sensor_msgs::msg::NavSatFix rtk_msg_;

 public:
  DJISubscriptionRTK(as2::Node *node, Vehicle *vehicle, int frequency = 50,
                     bool enable_timestamp = false)
      : DJISubscription("RTK", node, vehicle, frequency, enable_timestamp),
        rtk_("rtk", node){};

 protected:
  void initializeTopics() override {
    topics_ = {TOPIC_RTK_CONNECT_STATUS, TOPIC_RTK_POSITION, TOPIC_RTK_YAW_INFO,
               TOPIC_RTK_POSITION_INFO,  TOPIC_RTK_VELOCITY, TOPIC_RTK_YAW};
  };

  void onUpdate() override {
    TypeMap<TOPIC_RTK_CONNECT_STATUS>::type rtk_connect_status;
    TypeMap<TOPIC_RTK_POSITION>::type rtk;
    TypeMap<TOPIC_RTK_POSITION_INFO>::type rtk_pos_info;
    TypeMap<TOPIC_RTK_VELOCITY>::type rtk_velocity;
    TypeMap<TOPIC_RTK_YAW>::type rtk_yaw;
    TypeMap<TOPIC_RTK_YAW_INFO>::type rtk_yaw_info;

    rtk_connect_status =
        vehicle_->subscribe->getValue<TOPIC_RTK_CONNECT_STATUS>();
    rtk = vehicle_->subscribe->getValue<TOPIC_RTK_POSITION>();
    rtk_pos_info = vehicle_->subscribe->getValue<TOPIC_RTK_POSITION_INFO>();
    rtk_velocity = vehicle_->subscribe->getValue<TOPIC_RTK_VELOCITY>();
    rtk_yaw = vehicle_->subscribe->getValue<TOPIC_RTK_YAW>();
    rtk_yaw_info = vehicle_->subscribe->getValue<TOPIC_RTK_YAW_INFO>();

    // TODO: Check this if
    if (rtk_connect_status.rtkConnected) {
      // std::cout << "RTK if available   "
      //              "(lat/long/alt/velocity_x/velocity_y/velocity_z/yaw/yaw_info/pos_info)
      //              ="
      //           << rtk.latitude << "," << rtk.longitude << "," << rtk.HFSL <<
      //           "," << rtk_velocity.x
      //           << "," << rtk_velocity.y << "," << rtk_velocity.z << "," <<
      //           rtk_yaw << ","
      //           << (uint16_t)rtk_yaw_info << "," << (uint16_t)rtk_pos_info <<
      //           "\n";

      rtk_msg_.header.stamp = node_->now();
      rtk_msg_.header.frame_id = "wgs84";
      rtk_msg_.status.status = rtk_connect_status.rtkConnected;
      rtk_msg_.status.service = 0;  // FIXME: what is this?
      rtk_msg_.latitude = rtk.latitude;
      rtk_msg_.longitude = rtk.longitude;
      rtk_msg_.altitude = rtk.HFSL;
      // COVARIANCE UNKNOWN
      // rtk_msg_.position_covariance[0] =
      // rtk_msg_.position_covariance[4] =
      // rtk_msg_.position_covariance[8 =
      rtk_msg_.position_covariance_type = 0;
      rtk_.updateData(rtk_msg_);
    }
  }
};

class DJISubscriptionCompass : public DJISubscription {
 private:
  as2::sensors::Compass compass_;
  sensor_msgs::msg::MagneticField compass_msg_;

 public:
  DJISubscriptionCompass(as2::Node *node, Vehicle *vehicle, int frequency = 100,
                         bool enable_timestamp = false)
      : DJISubscription("Compass", node, vehicle, frequency, enable_timestamp),
        compass_("compass", node){};

 protected:
  void initializeTopics() override { topics_ = {TOPIC_COMPASS}; };

  void onUpdate() override {
    TypeMap<TOPIC_COMPASS>::type compass;
    compass = vehicle_->subscribe->getValue<TOPIC_COMPASS>();
    compass_msg_.header.stamp = node_->now();
    compass_msg_.header.frame_id =
        as2::tf::generateTfName(node_->get_namespace(), "imu");
    compass_msg_.magnetic_field.x = compass.x;
    compass_msg_.magnetic_field.y = compass.y;
    compass_msg_.magnetic_field.z = compass.z;
    compass_.updateData(compass_msg_);
  }
};

// TODO: CHANGE THIS SUBSCRIPTION TO HARDWARE_SYNC
class DJISubscriptionImu : public DJISubscription {
  as2::sensors::Imu imu_;
  sensor_msgs::msg::Imu imu_msg_;

 public:
  DJISubscriptionImu(as2::Node *node, Vehicle *vehicle, int frequency = 200,
                     bool enable_timestamp = false)
      : DJISubscription("FlightStatus", node, vehicle, frequency,
                        enable_timestamp),
        imu_("imu", node){};

 protected:
  void initializeTopics() override {
    topics_ = {TOPIC_ACCELERATION_BODY, TOPIC_ANGULAR_RATE_FUSIONED,
               TOPIC_QUATERNION};
  };

  void onUpdate() override {
    TypeMap<TOPIC_ACCELERATION_BODY>::type accel;
    TypeMap<TOPIC_ANGULAR_RATE_FUSIONED>::type angular_rate;
    TypeMap<TOPIC_QUATERNION>::type quaternion;

    accel = vehicle_->subscribe->getValue<TOPIC_ACCELERATION_BODY>();
    angular_rate = vehicle_->subscribe->getValue<TOPIC_ANGULAR_RATE_FUSIONED>();
    quaternion = vehicle_->subscribe->getValue<TOPIC_QUATERNION>();

    imu_msg_.header.stamp = node_->now();
    imu_msg_.header.frame_id =
        as2::tf::generateTfName(node_->get_namespace(), "imu");
    imu_msg_.orientation.w = quaternion.q0;
    imu_msg_.orientation.x = quaternion.q1;
    imu_msg_.orientation.y = quaternion.q2;
    imu_msg_.orientation.z = quaternion.q3;

    imu_msg_.linear_acceleration.x = accel.x;
    imu_msg_.linear_acceleration.y = accel.y;
    imu_msg_.linear_acceleration.z = accel.z;

    imu_msg_.angular_velocity.x = angular_rate.x;
    imu_msg_.angular_velocity.y = angular_rate.y;
    imu_msg_.angular_velocity.z = angular_rate.z;

    // TODO: Check frame coordinates
    imu_.updateData(imu_msg_);
  };
};

class DJISubscriptionBattery : public DJISubscription {
  as2::sensors::Battery battery_;
  sensor_msgs::msg::BatteryState battery_msg_;

 public:
  DJISubscriptionBattery(as2::Node *node, Vehicle *vehicle, int frequency = 5,
                         bool enable_timestamp = false)
      : DJISubscription("Battery", node, vehicle, frequency, enable_timestamp),
        battery_("battery", node){};

 protected:
  void initializeTopics() override { topics_ = {TOPIC_BATTERY_INFO}; };

  void onUpdate() override {
    TypeMap<TOPIC_BATTERY_INFO>::type battery;
    battery = vehicle_->subscribe->getValue<TOPIC_BATTERY_INFO>();

    battery_msg_.header.stamp = node_->now();
    battery_msg_.voltage = battery.voltage / 1000.0f;
    battery_msg_.current = battery.current / 1000.0f;
    battery_msg_.capacity = battery.capacity / 1000.0f;
    battery_msg_.percentage = battery.percentage / 100.0f;
    battery_.updateData(battery_msg_);
  };
};

class DJISubscriptionFlightStatus : public DJISubscription {
 private:
  uint8_t flight_status_ = 0;

 public:
  DJISubscriptionFlightStatus(as2::Node *node, Vehicle *vehicle,
                              int frequency = 10, bool enable_timestamp = false)
      : DJISubscription("FlightStatus", node, vehicle, frequency,
                        enable_timestamp){};

 protected:
  void initializeTopics() override { topics_ = {TOPIC_STATUS_FLIGHT}; };

  void onUpdate() override {
    TypeMap<TOPIC_STATUS_FLIGHT>::type flight_status;
    flight_status = vehicle_->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    if (flight_status != flight_status_) {
      flight_status_ = flight_status;
      RCLCPP_INFO(node_->get_logger(), "DJI Flight status changed to %d",
                  flight_status_);
    }
    flight_status_ = flight_status;
  };

  // TODO: integrate it better with the rest of the code
  unsigned char getFlightStatus() { return flight_status_; }
};

class DJISubscriptionOdometry : public DJISubscription {
 private:
  as2::sensors::Sensor<nav_msgs::msg::Odometry> odom_;
  nav_msgs::msg::Odometry odom_msg_;
  as2::gps::GpsHandler gps_handler_;
  bool is_gps_initialized_ = false;
  as2::sensors::GPS gps_;
  sensor_msgs::msg::NavSatFix gps_msg_;

 public:
  DJISubscriptionOdometry(as2::Node *node, Vehicle *vehicle, int frequency = 50,
                          bool enable_timestamp = false)
      : DJISubscription("Odometry", node, vehicle, frequency, enable_timestamp),
        odom_("odom", node),
        gps_("gps", node){};

 protected:
  void initializeTopics() override {
    topics_ = {TOPIC_ALTITUDE_FUSIONED, TOPIC_GPS_FUSED, TOPIC_QUATERNION,
               TOPIC_VELOCITY};
  };

  void onUpdate() override {
    TypeMap<TOPIC_GPS_FUSED>::type gps;
    TypeMap<TOPIC_QUATERNION>::type quaternion;
    TypeMap<TOPIC_VELOCITY>::type velocity;
    TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
    gps = vehicle_->subscribe->getValue<TOPIC_GPS_FUSED>();
    quaternion = vehicle_->subscribe->getValue<TOPIC_QUATERNION>();
    velocity = vehicle_->subscribe->getValue<TOPIC_VELOCITY>();
    altitude = vehicle_->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();

    // FIXME: This is a hack to get the GPS to work.
    gps.altitude = altitude;

    if (!is_gps_initialized_) {
      if (gps.visibleSatelliteNumber < 4) {
        RCLCPP_WARN_ONCE(node_->get_logger(), "DJI GPS not initialized");
        return;
      }
      RCLCPP_WARN_ONCE(node_->get_logger(), "DJI GPS initialized");
      gps_handler_.setOrigin(gps.latitude * 180.0 / M_PI,
                             gps.longitude * 180.0 / M_PI, gps.altitude);
      is_gps_initialized_ = true;
    }

    gps_msg_.header.stamp = node_->now();
    gps_msg_.header.frame_id = "wgs84";
    gps_msg_.latitude = gps.latitude * 180 / M_PI;
    gps_msg_.longitude = gps.longitude * 180 / M_PI;
    gps_msg_.altitude = gps.altitude;
    gps_msg_.position_covariance_type = 0;
    gps_.updateData(gps_msg_);

    double x, y, z;
    gps_handler_.LatLon2Local(gps.latitude * 180.0 / M_PI,
                              gps.longitude * 180.0 / M_PI, gps.altitude, x, y,
                              z);
    odom_msg_.header.stamp = node_->now();
    odom_msg_.header.frame_id =
        as2::tf::generateTfName(node_->get_namespace(), "odom");
    odom_msg_.child_frame_id =
        as2::tf::generateTfName(node_->get_namespace(), "base_link");
    // DJI pose frame is in NED coordinate system, so we need to convert to ENU
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = y;
    odom_msg_.pose.pose.position.z = z;

    odom_msg_.pose.pose.orientation.x = quaternion.q1;
    odom_msg_.pose.pose.orientation.y = quaternion.q2;
    odom_msg_.pose.pose.orientation.z = quaternion.q3;
    odom_msg_.pose.pose.orientation.w = quaternion.q0;

    double roll_frd, pitch_frd, yaw_frd;
    as2::frame::quaternionToEuler(odom_msg_.pose.pose.orientation, roll_frd,
                                  pitch_frd, yaw_frd);

    // convert orientation from FRD to FLU
    double roll_flu, pitch_flu, yaw_flu;

    roll_flu = roll_frd;
    pitch_flu = -pitch_frd;
    yaw_flu = -yaw_frd + M_PI_2;

    as2::frame::eulerToQuaternion(roll_flu, pitch_flu, yaw_flu,
                                  odom_msg_.pose.pose.orientation);

    Eigen::Vector3d vel_NEU(velocity.data.x, velocity.data.y, velocity.data.z);
    Eigen::Vector3d vel_ENU =
        Eigen::Vector3d(vel_NEU.y(), vel_NEU.x(), vel_NEU.z());

    // convert ENU to FLU
    auto flu_speed =
        as2::frame::transform(odom_msg_.pose.pose.orientation, vel_ENU);
    odom_msg_.twist.twist.linear.x = flu_speed.x();
    odom_msg_.twist.twist.linear.y = flu_speed.y();
    odom_msg_.twist.twist.linear.z = flu_speed.z();

    odom_.updateData(odom_msg_);
  };
};

#endif
