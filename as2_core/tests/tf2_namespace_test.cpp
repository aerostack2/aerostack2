// Copyright 2023 Universidad Politécnica de Madrid
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

// #include <iostream>
// #include <stdexcept>

#include "gtest/gtest.h"
#include "utils/tf_utils.hpp"

TEST(TF2NamespacesTest, empty_empty) {
  EXPECT_THROW(as2::tf::generateTfName("", ""), std::runtime_error);
}
TEST(TF2NamespacesTest, ns_empty) {
  EXPECT_THROW(as2::tf::generateTfName("ns", ""), std::runtime_error);
}

TEST(TF2NamespacesTest, ns_foo) {
  EXPECT_EQ(as2::tf::generateTfName("ns", "foo"), "ns/foo");
}
TEST(TF2NamespacesTest, empty_foo) {
  EXPECT_EQ(as2::tf::generateTfName("", "foo"), "foo");
}

TEST(TF2NamespacesTest, ns_global_foo) {
  EXPECT_EQ(as2::tf::generateTfName("ns", "/foo"), "foo");
}
TEST(TF2NamespacesTest, empty_global_foo) {
  EXPECT_EQ(as2::tf::generateTfName("", "/foo"), "foo");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
