// Copyright 2026 Universidad Politécnica de Madrid
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

/*!*******************************************************************************************
 *  \file       coordinate_item.hpp
 *  \brief      AuctionItemPlugin for a 3D target coordinate header
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef COORDINATE_ITEM__COORDINATE_ITEM_HPP_
#define COORDINATE_ITEM__COORDINATE_ITEM_HPP_

#include <memory>
#include <string>

#include "as2_auction_behavior/auction_item_plugin_base.hpp"
#include "as2_msgs/msg/auction_item.hpp"

namespace coordinate_item
{

class Plugin : public as2_auction_behavior::AuctionItemPluginBase
{
public:
  Plugin() = default;

  std::shared_ptr<AuctionItemPluginBase> create(
    const as2_msgs::msg::AuctionItem & item_msg) const override;

  float evaluate(const StateInterface & state_interface) const override;

  std::string get_name() const override;

  std::string to_string() const override;

  as2_msgs::msg::AuctionItem get_item() const override;

private:
  as2_msgs::msg::AuctionItem item_;
  std::string name_;
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
};

}  // namespace coordinate_item

#endif  // COORDINATE_ITEM__COORDINATE_ITEM_HPP_
