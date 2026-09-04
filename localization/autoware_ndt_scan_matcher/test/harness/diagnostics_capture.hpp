// Copyright 2026 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HARNESS__DIAGNOSTICS_CAPTURE_HPP_
#define HARNESS__DIAGNOSTICS_CAPTURE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ndt_test
{

/// Records everything published on `/diagnostics`, indexed by `status[0].name`.
///
/// The assertions rely on two details of `DiagnosticsInterface`: values become strings via
/// `std::to_string`, with bools as "True"/"False", and `values` keeps insertion order, so the key
/// order shows which path ran. The subscription is `KeepAll().reliable()` because the node
/// publishes with a volatile `QoS(10)`, which a shallower subscription would drop.
class DiagnosticsCapture
{
public:
  /// One captured `DiagnosticArray`. Use `value()` for equality and `value_as_double()` for
  /// ordering: a missing key gives "" for the first and NaN for the second, so both fail.
  class Record
  {
  public:
    Record() = default;
    explicit Record(
      diagnostic_msgs::msg::DiagnosticStatus status, builtin_interfaces::msg::Time stamp)
    : status_(std::move(status)), stamp_(stamp)
    {
    }

    /// Keys in the order `DiagnosticsInterface::add_key_value` inserted them.
    [[nodiscard]] std::vector<std::string> keys_in_order() const
    {
      std::vector<std::string> keys;
      keys.reserve(status_.values.size());
      for (const auto & kv : status_.values) {
        keys.push_back(kv.key);
      }
      return keys;
    }

    [[nodiscard]] bool has_key(const std::string & key) const
    {
      return std::any_of(status_.values.begin(), status_.values.end(), [&](const auto & kv) {
        return kv.key == key;
      });
    }

    /// The raw (stringified) value, or "" when the key is absent.
    [[nodiscard]] std::string value(const std::string & key) const
    {
      const auto it = std::find_if(
        status_.values.begin(), status_.values.end(),
        [&](const auto & kv) { return kv.key == key; });
      return (it == status_.values.end()) ? std::string{} : it->value;
    }

    /// Parsed value, or NaN when the key is absent. Not for `topic_time_stamp`, which is too large.
    [[nodiscard]] double value_as_double(const std::string & key) const
    {
      return has_key(key) ? std::stod(value(key)) : std::numeric_limits<double>::quiet_NaN();
    }

    [[nodiscard]] int8_t level() const { return status_.level; }
    [[nodiscard]] const std::string & message() const { return status_.message; }
    [[nodiscard]] const builtin_interfaces::msg::Time & stamp() const { return stamp_; }

  private:
    diagnostic_msgs::msg::DiagnosticStatus status_{};
    builtin_interfaces::msg::Time stamp_{};
  };

  /// `observer` owns the subscription and must be spun by the caller.
  explicit DiagnosticsCapture(rclcpp::Node * observer)
  {
    subscription_ = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepAll()).reliable(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) { on_message(*msg); });
  }

  /// Publisher count, used as the discovery gate: one publisher per `DiagnosticsInterface`.
  [[nodiscard]] size_t publisher_count() const { return subscription_->get_publisher_count(); }

  [[nodiscard]] std::vector<Record> records(const std::string & status_name) const
  {
    const std::lock_guard<std::mutex> lock(records_mutex_);
    const auto it = records_.find(status_name);
    return (it == records_.end()) ? std::vector<Record>{} : it->second;
  }

  [[nodiscard]] size_t count(const std::string & status_name) const
  {
    const std::lock_guard<std::mutex> lock(records_mutex_);
    const auto it = records_.find(status_name);
    return (it == records_.end()) ? 0U : it->second.size();
  }

  /// Remembers the current count, so `newest_since_mark` sees only what arrives after it.
  void mark(const std::string & status_name) { marks_[status_name] = count(status_name); }

  [[nodiscard]] std::optional<Record> newest_since_mark(const std::string & status_name) const
  {
    const size_t mark = marks_.count(status_name) ? marks_.at(status_name) : 0U;
    const auto all = records(status_name);
    if (all.size() <= mark) {
      return std::nullopt;
    }
    return all.back();
  }

  /// Finds the record whose `header.stamp` equals `stamp`, which identifies it exactly.
  [[nodiscard]] std::optional<Record> find_by_stamp(
    const std::string & status_name, const builtin_interfaces::msg::Time & stamp) const
  {
    for (const auto & record : records(status_name)) {
      if (record.stamp().sec == stamp.sec && record.stamp().nanosec == stamp.nanosec) {
        return record;
      }
    }
    return std::nullopt;
  }

private:
  void on_message(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    const std::lock_guard<std::mutex> lock(records_mutex_);
    for (const auto & status : msg.status) {
      records_[status.name].emplace_back(status, msg.header.stamp);
    }
  }

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  mutable std::mutex records_mutex_;
  std::map<std::string, std::vector<Record>> records_;
  std::map<std::string, size_t> marks_;
};

}  // namespace ndt_test

#endif  // HARNESS__DIAGNOSTICS_CAPTURE_HPP_
