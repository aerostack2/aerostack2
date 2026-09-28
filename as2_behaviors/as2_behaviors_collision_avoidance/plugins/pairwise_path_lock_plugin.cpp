#include "as2_behaviors_collision_avoidance/collision_avoidance_base.hpp"
#include "as2_behaviors_collision_avoidance/path_geometry.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <deque>
#include <mutex>
#include <set>

namespace pairwise_path_lock_plugin {

using Request = as2_msgs::msg::CAPathLockRequest;
using Grant   = as2_msgs::msg::CAPathLockGrant;
using Release = as2_msgs::msg::CAPathLockRelease;
using Status  = collision_avoidance_base::Status;
using PoseWithID = as2_msgs::msg::PoseStampedWithID;

struct DeferredReq {
  std::string peer_id;
  uint32_t    req_id;
  std::vector<PoseWithID> path;
};

class Plugin : public collision_avoidance_base::CollisionAvoidanceBase {
public:

  void initialize(rclcpp::Node * node,
                  std::shared_ptr<as2_ca::CAGatewayClient> ca,
                  const std::string & own_id) override
  {
    node_   = node;
    ca_     = ca;
    own_id_ = own_id;
    state_  = "IDLE";
  }

  // ── Called by server when the action goal is accepted ───────────────────────
  void start_acquisition(const std::vector<PoseWithID> & path,
                         const std::vector<std::string> & peers,
                         uint32_t req_id,
                         double safety_distance) override
  {
    std::lock_guard<std::mutex> lk(mu_);
    own_path_        = path;
    safety_distance_ = safety_distance;
    req_id_          = req_id;
    pending_peers_   = std::set<std::string>(peers.begin(), peers.end());
    grants_received_.clear();
    state_ = "REQUESTING";

    if (pending_peers_.empty()) {
      // No peers — immediately hold.
      state_ = "HOLDING";
      return;
    }

    // Broadcast lock request to all peers.
    Request msg;
    msg.header.stamp = node_->now();
    msg.requester_id = own_id_;
    msg.req_id       = req_id_;
    msg.path         = own_path_;

    for (const auto & peer : pending_peers_) {
      ca_->forward_IA_msg<Request>(msg, "ca_lock_request", {peer});
    }
  }

  // ── Called by server on action cancel / deactivate ──────────────────────────
  void release() override
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (state_ == "IDLE") return;

    state_ = "RELEASING";

    // Notify all known peers that we are releasing.
    Release rel;
    rel.releaser_id = own_id_;
    rel.req_id      = req_id_;
    ca_->forward_IA_msg<Release>(rel, "ca_lock_release", all_known_peers());

    // Flush all deferred grants.
    flush_deferred_locked();

    // Clear peer lock knowledge gained during this round.
    peer_held_paths_.clear();
    pending_peers_.clear();
    grants_received_.clear();
    state_ = "IDLE";
  }

  // ── Incoming peer messages ──────────────────────────────────────────────────
  void on_request(const Request & req, const std::string & /*sender*/) override
  {
    std::lock_guard<std::mutex> lk(mu_);

    if (req.requester_id == own_id_) return;  // own echo

    bool conflicts = path_geometry::min_polyline_distance(own_path_, req.path)
                     < safety_distance_;

    bool should_grant = true;

    if (state_ == "HOLDING" && conflicts) {
      should_grant = false;  // defer until we release
    } else if (state_ == "REQUESTING" && conflicts) {
      // Tie-break: lower lex id has priority.
      if (req.requester_id < own_id_) {
        // Peer has priority — grant immediately.
        should_grant = true;
      } else {
        // We have priority — defer.
        should_grant = false;
      }
    }
    // state == IDLE or no conflict → grant.

    if (should_grant) {
      send_grant_locked(req.requester_id, req.req_id);
      peer_held_paths_[req.requester_id] = req.path;
    } else {
      deferred_.push_back({req.requester_id, req.req_id, req.path});
    }
  }

  void on_grant(const Grant & grant, const std::string & /*sender*/) override
  {
    std::lock_guard<std::mutex> lk(mu_);

    if (state_ != "REQUESTING") return;
    if (grant.requester_id != own_id_) return;
    if (grant.req_id != req_id_) return;

    grants_received_.insert(grant.granter_id);

    if (std::includes(
        grants_received_.begin(), grants_received_.end(),
        pending_peers_.begin(), pending_peers_.end()))
    {
      state_ = "HOLDING";
    }
  }

  void on_release(const Release & rel, const std::string & /*sender*/) override
  {
    std::lock_guard<std::mutex> lk(mu_);

    if (rel.releaser_id == own_id_) return;

    peer_held_paths_.erase(rel.releaser_id);

    // If we deferred a request from this peer, grant it now.
    // (If peer released without a hold in our view, it may have cancelled —
    //  remove from deferred without granting.)
    auto it = deferred_.begin();
    while (it != deferred_.end()) {
      if (it->peer_id == rel.releaser_id) {
        // Peer released; this means the request it sent before is now stale —
        // we do NOT grant it (the peer already freed itself).
        it = deferred_.erase(it);
      } else {
        ++it;
      }
    }

    // Also remove from pending_peers_ if we had not yet received a grant from them.
    // (Peer crashed / cancelled before granting us; treat as implicit grant.)
    if (state_ == "REQUESTING") {
      pending_peers_.erase(rel.releaser_id);
      if (std::includes(
          grants_received_.begin(), grants_received_.end(),
          pending_peers_.begin(), pending_peers_.end()))
      {
        state_ = "HOLDING";
      }
    }
  }

  Status status() const override
  {
    std::lock_guard<std::mutex> lk(mu_);
    Status s;
    s.state    = state_;
    s.lock_held = (state_ == "HOLDING");

    if (state_ == "REQUESTING") {
      for (const auto & p : pending_peers_) {
        if (grants_received_.count(p)) {
          s.granted_peers.push_back(p);
        } else {
          s.pending_peers.push_back(p);
        }
      }
    }
    for (const auto & [id, path] : peer_held_paths_) {
      if (path_geometry::min_polyline_distance(own_path_, path) < safety_distance_) {
        s.conflicting_peers.push_back(id);
      }
    }
    s.deferred_count = static_cast<uint32_t>(deferred_.size());
    return s;
  }

private:

  void send_grant_locked(const std::string & to, uint32_t req_id)
  {
    Grant g;
    g.granter_id   = own_id_;
    g.requester_id = to;
    g.req_id       = req_id;
    ca_->forward_IA_msg<Grant>(g, "ca_lock_grant", {to});
  }

  void flush_deferred_locked()
  {
    for (const auto & d : deferred_) {
      send_grant_locked(d.peer_id, d.req_id);
    }
    deferred_.clear();
  }

  std::vector<std::string> all_known_peers() const
  {
    std::vector<std::string> out;
    for (const auto & p : pending_peers_)      out.push_back(p);
    for (const auto & [id, _] : peer_held_paths_) {
      if (!std::count(out.begin(), out.end(), id)) out.push_back(id);
    }
    return out;
  }

  // ── State ────────────────────────────────────────────────────────────────────
  mutable std::mutex mu_;
  rclcpp::Node * node_{nullptr};
  std::shared_ptr<as2_ca::CAGatewayClient> ca_;
  std::string own_id_;
  std::string state_{"IDLE"};

  std::vector<PoseWithID>  own_path_;
  double               safety_distance_{1.5};
  uint32_t             req_id_{0};

  std::set<std::string>                      pending_peers_;
  std::set<std::string>                      grants_received_;
  std::unordered_map<std::string, std::vector<PoseWithID>> peer_held_paths_;
  std::deque<DeferredReq>                    deferred_;
};

}  // namespace pairwise_path_lock_plugin

PLUGINLIB_EXPORT_CLASS(
  pairwise_path_lock_plugin::Plugin,
  collision_avoidance_base::CollisionAvoidanceBase)
