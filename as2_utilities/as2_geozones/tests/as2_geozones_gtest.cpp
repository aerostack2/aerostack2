// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file as2_geozones_gtest.cpp
*
* as2_geozones test file.
*
* @author Javilinos
*/

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <as2_geozones/as2_geozones.hpp>
#include <as2_msgs/srv/set_geozone.hpp>
#include <as2_msgs/srv/get_geozone.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

// ---------------------------------------------------------------------------
// Helper: fake get_origin service node
// ---------------------------------------------------------------------------

/// Spins a minimal rclcpp::Node that advertises the get_origin service and
/// responds with a fixed origin (Madrid city centre).
class FakeOriginServer
{
public:
  explicit FakeOriginServer(bool respond_success = true)
  : node_(std::make_shared<rclcpp::Node>("fake_origin_server")),
    respond_success_(respond_success)
  {
    using GetOrigin = as2_msgs::srv::GetOrigin;
    srv_ = node_->create_service<GetOrigin>(
      "/get_origin",
      [this](
        const std::shared_ptr<GetOrigin::Request> /*req*/,
        std::shared_ptr<GetOrigin::Response> res)
      {
        res->success = respond_success_;
        if (respond_success_) {
          res->origin.latitude = 40.4520;
          res->origin.longitude = -3.7274;
          res->origin.altitude = 650.0;
        }
      });
    spin_thread_ = std::thread([this]() {rclcpp::spin(node_);});
  }

  ~FakeOriginServer()
  {
    rclcpp::shutdown();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<as2_msgs::srv::GetOrigin>::SharedPtr srv_;
  bool respond_success_;
  std::thread spin_thread_;
};

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class GeozonesTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(GeozonesTest, ConstructorDoesNotThrow) {
  EXPECT_NO_THROW(
    std::shared_ptr<Geozones> node =
    std::make_shared<Geozones>());
}

/// Verify that adding a Cartesian geozone via set_geozone succeeds and that
/// the polygon points are stored verbatim (no coordinate conversion).
TEST(GeozonesTest, SetCartesianGeozoneStoresRawPoints) {
  auto node = std::make_shared<Geozones>();

  as2_msgs::srv::SetGeozone::Request::SharedPtr req =
    std::make_shared<as2_msgs::srv::SetGeozone::Request>();
  req->geozone.id = 10;
  req->geozone.type = "geofence";
  req->geozone.data_type = "cartesian";
  req->geozone.alert = 1;
  req->geozone.z_up = 100.0f;
  req->geozone.z_down = 0.0f;

  geometry_msgs::msg::Point32 p;
  p.x = 1.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);
  p.x = 2.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);
  p.x = 1.5f; p.y = 1.0f; req->geozone.polygon.points.push_back(p);

  auto res = std::make_shared<as2_msgs::srv::SetGeozone::Response>();

  // Call the callback directly using the node's exposed method via friend or
  // by constructing a service client. Because the node is not spinning we
  // invoke the callback in the same thread.
  node->setGeozoneCbForTest(req, res);

  EXPECT_TRUE(res->success);

  // Retrieve and verify the stored points.
  auto get_req = std::make_shared<as2_msgs::srv::GetGeozone::Request>();
  auto get_res = std::make_shared<as2_msgs::srv::GetGeozone::Response>();
  node->getGeozoneCbForTest(get_req, get_res);

  ASSERT_TRUE(get_res->success);
  ASSERT_EQ(get_res->geozone_list.size(), 1u);
  ASSERT_EQ(get_res->geozone_list[0].polygon.points.size(), 3u);
  EXPECT_FLOAT_EQ(get_res->geozone_list[0].polygon.points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(get_res->geozone_list[0].polygon.points[1].x, 2.0f);
}

/// Verify that get_geozone returns success=false and does NOT crash when no
/// GPS origin has been established and a GPS geozone is queried.
/// This is the null-pointer regression test for issue #1005.
TEST(GeozonesTest, GetGeozoneGpsWithoutOriginReturnsFailure) {
  auto node = std::make_shared<Geozones>();

  // Manually inject a GPS geozone with raw lat/lon values (simulating the
  // state the node was in before this fix).
  as2_msgs::srv::SetGeozone::Request::SharedPtr req =
    std::make_shared<as2_msgs::srv::SetGeozone::Request>();
  req->geozone.id = 20;
  req->geozone.type = "geocage";
  req->geozone.data_type = "cartesian";   // store as cartesian to bypass GPS init
  req->geozone.alert = 2;
  req->geozone.z_up = 50.0f;
  req->geozone.z_down = 0.0f;

  geometry_msgs::msg::Point32 p;
  p.x = 0.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);
  p.x = 5.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);
  p.x = 2.5f; p.y = 5.0f; req->geozone.polygon.points.push_back(p);

  auto set_res = std::make_shared<as2_msgs::srv::SetGeozone::Response>();
  node->setGeozoneCbForTest(req, set_res);
  ASSERT_TRUE(set_res->success);

  // Now artificially override the data_type in the stored geozone to "gps"
  // without setting gps_handler. getGeozoneCb must not crash.
  node->forceGeozoneDataTypeForTest(20, "gps");

  auto get_req = std::make_shared<as2_msgs::srv::GetGeozone::Request>();
  auto get_res = std::make_shared<as2_msgs::srv::GetGeozone::Response>();

  EXPECT_NO_FATAL_FAILURE(node->getGeozoneCbForTest(get_req, get_res));
  EXPECT_FALSE(get_res->success);
}

/// Verify that adding a geozone with an id that already exists is rejected.
TEST(GeozonesTest, DuplicateIdIsRejected) {
  auto node = std::make_shared<Geozones>();

  auto make_req = [](int id) {
      auto req = std::make_shared<as2_msgs::srv::SetGeozone::Request>();
      req->geozone.id = id;
      req->geozone.type = "geofence";
      req->geozone.data_type = "cartesian";
      req->geozone.alert = 1;
      geometry_msgs::msg::Point32 p;
      p.x = 0.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);
      p.x = 1.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);
      p.x = 0.5f; p.y = 1.0f; req->geozone.polygon.points.push_back(p);
      return req;
    };

  auto res1 = std::make_shared<as2_msgs::srv::SetGeozone::Response>();
  node->setGeozoneCbForTest(make_req(5), res1);
  EXPECT_TRUE(res1->success);

  auto res2 = std::make_shared<as2_msgs::srv::SetGeozone::Response>();
  node->setGeozoneCbForTest(make_req(5), res2);
  EXPECT_FALSE(res2->success);
}

/// Verify that polygons with fewer than three points are rejected.
TEST(GeozonesTest, PolygonTooSmallIsRejected) {
  auto node = std::make_shared<Geozones>();

  auto req = std::make_shared<as2_msgs::srv::SetGeozone::Request>();
  req->geozone.id = 30;
  req->geozone.type = "geofence";
  req->geozone.data_type = "cartesian";
  req->geozone.alert = 1;
  // Only 2 points -- must be rejected.
  geometry_msgs::msg::Point32 p;
  p.x = 0.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);
  p.x = 1.0f; p.y = 0.0f; req->geozone.polygon.points.push_back(p);

  auto res = std::make_shared<as2_msgs::srv::SetGeozone::Response>();
  node->setGeozoneCbForTest(req, res);
  EXPECT_FALSE(res->success);
}

/// Verify that get_geozone returns false when no geozones have been set.
TEST(GeozonesTest, GetGeozoneWithNoGeozonesReturnsFailure) {
  auto node = std::make_shared<Geozones>();

  auto get_req = std::make_shared<as2_msgs::srv::GetGeozone::Request>();
  auto get_res = std::make_shared<as2_msgs::srv::GetGeozone::Response>();
  node->getGeozoneCbForTest(get_req, get_res);

  EXPECT_FALSE(get_res->success);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
