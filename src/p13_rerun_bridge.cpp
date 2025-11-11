#include <rerun.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/create_generic_subscription.hpp>
#include <rclcpp/detail/get_typesupport_handle.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rcpputils/shared_library.hpp>
#include <rmw/rmw.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;

constexpr std::size_t kMaxSequencePreview = 16;

struct TopicConfig {
    std::string topic;
    std::string type_string;
    std::optional<rclcpp::QoS> qos;
};

struct FlattenResult {
    std::map<std::string, double> numeric;
    std::vector<std::string> texts;
};

struct SubscriptionContext {
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::shared_ptr<rcpputils::SharedLibrary> type_library;
    const rosidl_message_type_support_t * type_support{nullptr};
    const MessageMembers * members{nullptr};
};

std::string normalize_unit(std::string value) {
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char c) {
        return !std::isspace(c);
    }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char c) {
        return !std::isspace(c);
    }).base(), value.end());

    if (value.empty()) {
        return "P13";
    }
    while (!value.empty() && value.front() == '/') {
        value.erase(value.begin());
    }
    return value.empty() ? std::string("P13") : value;
}

std::string entity_path(const std::string & unit, const std::string & topic, const std::string & category) {
    if (topic.size() > unit.size() + 2 && topic[0] == '/' && topic.compare(1, unit.size(), unit) == 0 && topic[unit.size() + 1] == '/') {
        std::string relative = topic.substr(unit.size() + 2);
        return unit + "/" + category + "/" + relative;
    }
    std::string cleaned = topic;
    if (!cleaned.empty() && cleaned.front() == '/') {
        cleaned.erase(cleaned.begin());
    }
    return "global/" + category + "/" + cleaned;
}

const MessageMembers * load_members(const std::string & type_string, std::shared_ptr<rcpputils::SharedLibrary> & library_out, const rosidl_message_type_support_t *& type_support_out) {
    library_out = rclcpp::detail::get_typesupport_library(type_string, "rosidl_typesupport_introspection_cpp");
    type_support_out = rclcpp::detail::get_message_typesupport_handle(type_string, "rosidl_typesupport_introspection_cpp", *library_out);
    if (type_support_out == nullptr) {
        throw std::runtime_error("Failed to load type support for " + type_string);
    }
    auto * members = static_cast<const MessageMembers *>(type_support_out->data);
    if (members == nullptr) {
        throw std::runtime_error("Introspection data missing for " + type_string);
    }
    return members;
}

bool is_time_message(const MessageMembers & members) {
    return std::string_view(members.message_namespace_) == "builtin_interfaces::msg" && std::string_view(members.message_name_) == "Time";
}

const void * member_data(const void * message, const MessageMember & member) {
    const auto * bytes = static_cast<const std::uint8_t *>(message);
    return bytes + member.offset_;
}

std::optional<double> numeric_from_basic(const MessageMember & member, const void * data) {
    switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
        return static_cast<double>(*reinterpret_cast<const float *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        return *reinterpret_cast<const double *>(data);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        return static_cast<double>(*reinterpret_cast<const long double *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
        return *reinterpret_cast<const bool *>(data) ? 1.0 : 0.0;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        return static_cast<double>(*reinterpret_cast<const std::int8_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        return static_cast<double>(*reinterpret_cast<const std::uint8_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        return static_cast<double>(*reinterpret_cast<const std::int16_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        return static_cast<double>(*reinterpret_cast<const std::uint16_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        return static_cast<double>(*reinterpret_cast<const std::int32_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        return static_cast<double>(*reinterpret_cast<const std::uint32_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        return static_cast<double>(*reinterpret_cast<const std::int64_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        return static_cast<double>(*reinterpret_cast<const std::uint64_t *>(data));
    default:
        return std::nullopt;
    }
}

std::string string_from_basic(const MessageMember & member, const void * data) {
    switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        return *reinterpret_cast<const std::string *>(data);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING: {
        const auto & w = *reinterpret_cast<const std::u16string *>(data);
        std::string converted;
        converted.reserve(w.size());
        for (char16_t ch : w) {
            converted.push_back(static_cast<char>(ch & 0xFF));
        }
        return converted;
    }
    default:
        return {};
    }
}

void flatten_primitive(const std::string & prefix, const MessageMember & member, const void * data, FlattenResult & out) {
    if (auto numeric = numeric_from_basic(member, data)) {
        out.numeric[prefix] = *numeric;
        return;
    }
    auto text = string_from_basic(member, data);
    if (!text.empty()) {
        std::ostringstream ss;
        ss << prefix << ": " << text;
        out.texts.push_back(ss.str());
        return;
    }
    std::ostringstream ss;
    ss << prefix << ": unsupported field";
    out.texts.push_back(ss.str());
}

void flatten_message(const std::string & prefix, const MessageMembers & members, const void * data, FlattenResult & out, std::size_t depth);

size_t primitive_size(uint8_t type_id) {
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8;

    switch (type_id) {
    case ROS_TYPE_BOOLEAN:
    case ROS_TYPE_CHAR:
    case ROS_TYPE_OCTET:
    case ROS_TYPE_UINT8:
    case ROS_TYPE_INT8:
        return sizeof(std::uint8_t);
    case ROS_TYPE_UINT16:
    case ROS_TYPE_INT16:
        return sizeof(std::uint16_t);
    case ROS_TYPE_UINT32:
    case ROS_TYPE_INT32:
    case ROS_TYPE_FLOAT32:
        return sizeof(std::uint32_t);
    case ROS_TYPE_UINT64:
    case ROS_TYPE_INT64:
    case ROS_TYPE_FLOAT64:
        return sizeof(std::uint64_t);
    case ROS_TYPE_DOUBLE:
        return sizeof(double);
    case ROS_TYPE_LONG_DOUBLE:
        return sizeof(long double);
    case ROS_TYPE_STRING:
        return sizeof(std::string);
    case ROS_TYPE_WSTRING:
        return sizeof(std::u16string);
    default:
        return 0;
    }
}

void flatten_sequence(const std::string & prefix, const MessageMember & member, const void * data, FlattenResult & out, std::size_t depth) {
    size_t size = member.size_function(const_cast<void *>(data));
    size_t limit = std::min<size_t>(size, kMaxSequencePreview);
    for (size_t idx = 0; idx < limit; ++idx) {
        const void * element = member.get_const_function(data, idx);
        std::string key = prefix.empty() ? std::to_string(idx) : prefix + "/" + std::to_string(idx);
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            const auto * sub_members = member.members_;
            flatten_message(key, *sub_members, element, out, depth + 1);
        } else {
            flatten_primitive(key, member, element, out);
        }
    }
    if (size > limit) {
        std::ostringstream ss;
        ss << prefix << ": sequence truncated at " << kMaxSequencePreview << " items";
        out.texts.push_back(ss.str());
    }
}

void flatten_message(const std::string & prefix, const MessageMembers & members, const void * data, FlattenResult & out, std::size_t depth) {
    if (is_time_message(members)) {
        const auto * sec_ptr = reinterpret_cast<const std::int32_t *>(static_cast<const std::uint8_t *>(data) + members.members_[0].offset_);
        const auto * nsec_ptr = reinterpret_cast<const std::uint32_t *>(static_cast<const std::uint8_t *>(data) + members.members_[1].offset_);
        double seconds = static_cast<double>(*sec_ptr) + static_cast<double>(*nsec_ptr) * 1e-9;
        out.numeric[prefix] = seconds;
        return;
    }

    const auto * base = static_cast<const std::uint8_t *>(data);
    for (uint32_t i = 0; i < members.member_count_; ++i) {
        const auto & child = members.members_[i];
        const void * child_ptr = base + child.offset_;
        std::string key = prefix.empty() ? child.name_ : prefix + "/" + child.name_;
        if (child.is_array_) {
            if (!child.is_upper_bound_ && child.array_size_ > 0) {
                if (child.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                    const auto * sub_members = child.members_;
                    for (size_t idx = 0; idx < child.array_size_; ++idx) {
                        const void * element = static_cast<const std::uint8_t *>(child_ptr) + idx * sub_members->size_of_;
                        std::string element_key = key + "/" + std::to_string(idx);
                        flatten_message(element_key, *sub_members, element, out, depth + 1);
                    }
                } else {
                    size_t elem_size = primitive_size(child.type_id_);
                    for (size_t idx = 0; idx < child.array_size_; ++idx) {
                        const void * element = static_cast<const std::uint8_t *>(child_ptr) + idx * elem_size;
                        std::string element_key = key + "/" + std::to_string(idx);
                        flatten_primitive(element_key, child, element, out);
                    }
                }
            } else {
                flatten_sequence(key, child, child_ptr, out, depth + 1);
            }
        } else if (child.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            const auto * sub_members = child.members_;
            flatten_message(key, *sub_members, child_ptr, out, depth + 1);
        } else {
            flatten_primitive(key, child, child_ptr, out);
        }
    }
}

void flatten_value(const std::string & prefix, const MessageMember & member, const void * data, FlattenResult & out, std::size_t depth) {
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        const auto * sub_members = member.members_;
        flatten_message(prefix, *sub_members, data, out, depth + 1);
        return;
    }
    if (member.is_array_) {
        if (!member.is_upper_bound_ && member.array_size_ > 0) {
            size_t elem_size = primitive_size(member.type_id_);
            const auto * base = static_cast<const std::uint8_t *>(data);
            for (size_t idx = 0; idx < member.array_size_; ++idx) {
                const void * element = base + idx * elem_size;
                std::string key = prefix.empty() ? std::to_string(idx) : prefix + "/" + std::to_string(idx);
                flatten_primitive(key, member, element, out);
            }
            return;
        }
        flatten_sequence(prefix, member, data, out, depth + 1);
        return;
    }
    flatten_primitive(prefix, member, data, out);
}

std::optional<double> extract_header_stamp(const MessageMembers & members, const void * data) {
    for (uint32_t idx = 0; idx < members.member_count_; ++idx) {
        const auto & member = members.members_[idx];
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE && std::string_view(member.name_) == "header" && !member.is_array_) {
            const auto * header_members = member.members_;
            if (header_members == nullptr) {
                continue;
            }
            const uint8_t * header_ptr = static_cast<const uint8_t *>(member_data(data, member));
            for (uint32_t j = 0; j < header_members->member_count_; ++j) {
                const auto & header_member = header_members->members_[j];
                if (header_member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE && std::string_view(header_member.name_) == "stamp") {
                    const auto * time_members = header_member.members_;
                    if (time_members == nullptr || !is_time_message(*time_members)) {
                        continue;
                    }
                    const uint8_t * stamp_ptr = header_ptr + header_member.offset_;
                    const auto * sec_ptr = reinterpret_cast<const std::int32_t *>(stamp_ptr + time_members->members_[0].offset_);
                    const auto * nsec_ptr = reinterpret_cast<const std::uint32_t *>(stamp_ptr + time_members->members_[1].offset_);
                    return static_cast<double>(*sec_ptr) + static_cast<double>(*nsec_ptr) * 1e-9;
                }
            }
        }
    }
    return std::nullopt;
}

void log_texts(rerun::RecordingStream & rec, const std::string & base_path, const std::vector<std::string> & texts) {
    if (texts.empty()) {
        return;
    }
    std::ostringstream ss;
    for (std::size_t i = 0; i < texts.size(); ++i) {
        if (i > 0) {
            ss << '\n';
        }
        ss << texts[i];
    }
    rec.log(base_path + "/text", rerun::TextLog(ss.str()));
}

void handle_as_scalars(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const MessageMembers & members, const void * data) {
    FlattenResult result;
    for (uint32_t idx = 0; idx < members.member_count_; ++idx) {
        const auto & member = members.members_[idx];
        const void * field_ptr = member_data(data, member);
        flatten_value(member.name_, member, field_ptr, result, 0);
    }

    const std::string base_path = entity_path(unit, topic, "graphs");
    for (const auto & [name, value] : result.numeric) {
        rec.log(base_path + "/" + name, rerun::Scalar(value));
    }
    log_texts(rec, base_path, result.texts);
}

void handle_twist(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const geometry_msgs::msg::Twist & msg) {
    const std::string base_path = entity_path(unit, topic, "graphs");
    rec.log(base_path + "/linear/x", rerun::Scalar(static_cast<double>(msg.linear.x)));
    rec.log(base_path + "/linear/y", rerun::Scalar(static_cast<double>(msg.linear.y)));
    rec.log(base_path + "/linear/z", rerun::Scalar(static_cast<double>(msg.linear.z)));
    rec.log(base_path + "/angular/x", rerun::Scalar(static_cast<double>(msg.angular.x)));
    rec.log(base_path + "/angular/y", rerun::Scalar(static_cast<double>(msg.angular.y)));
    rec.log(base_path + "/angular/z", rerun::Scalar(static_cast<double>(msg.angular.z)));
}

void handle_pose(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const geometry_msgs::msg::Point & translation, const geometry_msgs::msg::Quaternion & rotation) {
    rerun::Vec3D translation_vec(static_cast<double>(translation.x), static_cast<double>(translation.y), static_cast<double>(translation.z));
    rerun::Quaternion rotation_quat(static_cast<double>(rotation.x), static_cast<double>(rotation.y), static_cast<double>(rotation.z), static_cast<double>(rotation.w));
    rec.log(entity_path(unit, topic, "pose"), rerun::Transform3D(translation_vec, rotation_quat));
}

void handle_pose_message(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const geometry_msgs::msg::Pose & pose) {
    handle_pose(rec, unit, topic, pose.position, pose.orientation);
}

void handle_pose_stamped(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const geometry_msgs::msg::PoseStamped & pose) {
    handle_pose_message(rec, unit, topic, pose.pose);
}

void handle_point_stamped(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const geometry_msgs::msg::PointStamped & msg) {
    rerun::Vec3D point(static_cast<double>(msg.point.x), static_cast<double>(msg.point.y), static_cast<double>(msg.point.z));
    rec.log(entity_path(unit, topic, "sensors"), rerun::Points3D({point}));
}

void handle_odometry(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const nav_msgs::msg::Odometry & msg) {
    handle_pose(rec, unit, topic, msg.pose.pose.position, msg.pose.pose.orientation);

    const std::string base_path = entity_path(unit, topic, "graphs");
    rec.log(base_path + "/twist/linear/x", rerun::Scalar(static_cast<double>(msg.twist.twist.linear.x)));
    rec.log(base_path + "/twist/linear/y", rerun::Scalar(static_cast<double>(msg.twist.twist.linear.y)));
    rec.log(base_path + "/twist/linear/z", rerun::Scalar(static_cast<double>(msg.twist.twist.linear.z)));
    rec.log(base_path + "/twist/angular/x", rerun::Scalar(static_cast<double>(msg.twist.twist.angular.x)));
    rec.log(base_path + "/twist/angular/y", rerun::Scalar(static_cast<double>(msg.twist.twist.angular.y)));
    rec.log(base_path + "/twist/angular/z", rerun::Scalar(static_cast<double>(msg.twist.twist.angular.z)));
}

void handle_path(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const nav_msgs::msg::Path & msg) {
    std::vector<std::vector<double>> positions;
    positions.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
        positions.push_back({static_cast<double>(pose.pose.position.x), static_cast<double>(pose.pose.position.y), static_cast<double>(pose.pose.position.z)});
    }
    if (!positions.empty()) {
        std::vector<rerun::Vec3D> line;
        line.reserve(positions.size());
        for (const auto & p : positions) {
            line.emplace_back(p[0], p[1], p[2]);
        }
        rec.log(entity_path(unit, topic, "paths"), rerun::LineStrips3D({line}));
    }
}

void handle_string(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const std_msgs::msg::String & msg) {
    rec.log(entity_path(unit, topic, "logs"), rerun::TextLog(msg.data));
}

void handle_uint_scalar(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, std::uint64_t value) {
    rec.log(entity_path(unit, topic, "graphs"), rerun::Scalar(static_cast<double>(value)));
}

void handle_float64_multi_array(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const std_msgs::msg::Float64MultiArray & msg) {
    const std::string base_path = entity_path(unit, topic, "graphs");
    for (std::size_t idx = 0; idx < msg.data.size(); ++idx) {
        rec.log(base_path + "/" + std::to_string(idx), rerun::Scalar(static_cast<double>(msg.data[idx])));
    }
}

void handle_imu(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const sensor_msgs::msg::Imu & msg) {
    const std::string base_path = entity_path(unit, topic, "sensors");
    rerun::Quaternion orientation(static_cast<double>(msg.orientation.x), static_cast<double>(msg.orientation.y), static_cast<double>(msg.orientation.z), static_cast<double>(msg.orientation.w));
    rerun::Vec3D origin_translation(0.0, 0.0, 0.0);
    rec.log(base_path + "/orientation", rerun::Transform3D(origin_translation, orientation));

    rerun::Vec3D origin(0.0, 0.0, 0.0);
    rerun::Vec3D ang(static_cast<double>(msg.angular_velocity.x), static_cast<double>(msg.angular_velocity.y), static_cast<double>(msg.angular_velocity.z));
    rerun::Vec3D lin(static_cast<double>(msg.linear_acceleration.x), static_cast<double>(msg.linear_acceleration.y), static_cast<double>(msg.linear_acceleration.z));
    rec.log(base_path + "/angular_velocity", rerun::Arrows3D({origin}, {ang}));
    rec.log(base_path + "/linear_acceleration", rerun::Arrows3D({origin}, {lin}));
}

void handle_point_cloud(rerun::RecordingStream & rec, const std::string & unit, const std::string & topic, const sensor_msgs::msg::PointCloud2 & msg) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    std::vector<rerun::Vec3D> points;
    points.reserve(msg.width * msg.height);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            points.emplace_back(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
        }
    }

    if (!points.empty()) {
        rec.log(entity_path(unit, topic, "sensors"), rerun::Points3D(points));
    }
}

std::vector<TopicConfig> build_topic_list(const std::string & unit_name) {
    rclcpp::QoS sensor_qos(rclcpp::SensorDataQoS());

    const std::vector<std::tuple<std::string, std::string, std::optional<rclcpp::QoS>>> prefixed = {
        {"can/board_status", "pkr_msgs/msg/EcuSTAT", std::nullopt},
        {"can/eps_control", "pkr_msgs/msg/MotorCMD", std::nullopt},
        {"can/eps_status", "pkr_msgs/msg/MotorSTAT", std::nullopt},
        {"ctrl/task_request", "rtm_msgs/msg/IpkcComm", std::nullopt},
        {"dkc/task_request", "rtm_msgs/msg/IpkcComm", std::nullopt},
        {"driving_mode", "std_msgs/msg/UInt16", std::nullopt},
        {"err_hdlr/cmd", "std_msgs/msg/UInt8", std::nullopt},
        {"feedforward/motion_status", "dual_msgs/msg/MotionStatus", std::nullopt},
        {"filt_rel_pose_1", "geometry_msgs/msg/Pose", std::nullopt},
        {"front/ground_coeff", "nav_msgs/msg/Odometry", std::nullopt},
        {"imu", "sensor_msgs/msg/Imu", sensor_qos},
        {"itf/task_request", "rtm_msgs/msg/ItfComm", std::nullopt},
        {"mon_cmd", "nav_msgs/msg/Odometry", std::nullopt},
        {"monitoring/obstacle", "nav_msgs/msg/Odometry", std::nullopt},
        {"monitoring/task", "nav_msgs/msg/Odometry", std::nullopt},
        {"motion_Cmd", "geometry_msgs/msg/Twist", std::nullopt},
        {"obstacle_detection/critical_path", "nav_msgs/msg/Path", std::nullopt},
        {"obstacle_detection/obstacle_points", "sensor_msgs/msg/PointCloud2", sensor_qos},
        {"obstacle_detection/warning_path", "nav_msgs/msg/Path", std::nullopt},
        {"odom", "nav_msgs/msg/Odometry", std::nullopt},
        {"other_err_hdlr/error", "std_msgs/msg/UInt64", std::nullopt},
        {"other_obs_dualM", "geometry_msgs/msg/Twist", std::nullopt},
        {"other_obs_dual_motion", "geometry_msgs/msg/Twist", std::nullopt},
        {"other_tk_ctrl/init", "geometry_msgs/msg/PointStamped", std::nullopt},
        {"pose_COG", "nav_msgs/msg/Odometry", std::nullopt},
        {"pose_car", "geometry_msgs/msg/PoseStamped", std::nullopt},
        {"rear/ground_coeff", "nav_msgs/msg/Odometry", std::nullopt},
        {"rjh/task_request", "pkr_msgs/msg/RobotMissionTmp", std::nullopt},
        {"rjh/task_response", "std_msgs/msg/String", std::nullopt},
        {"rtm/stat", "rtm_msgs/msg/RtmStat", std::nullopt},
        {"rtm/task_result", "pkr_msgs/msg/Tm2ActrRSLT", std::nullopt},
        {"rtm/task_result_response", "std_msgs/msg/String", std::nullopt},
        {"target_and_now_wassy_angle", "std_msgs/msg/Float64MultiArray", std::nullopt},
        {"this/obs_dualM", "geometry_msgs/msg/Twist", std::nullopt},
        {"this/obs_dual_motion", "geometry_msgs/msg/Twist", std::nullopt},
        {"this/pose_COG", "nav_msgs/msg/Odometry", std::nullopt},
        {"this/tk_ctrl/init", "geometry_msgs/msg/PointStamped", std::nullopt},
        {"spatial_logmsg", "rcl_interfaces/msg/Log", std::nullopt},
    };

    const std::vector<std::tuple<std::string, std::string, std::optional<rclcpp::QoS>>> globals = {
        {"/diagnostics_agg", "diagnostic_msgs/msg/DiagnosticArray", std::nullopt},
        {"/hesai/pandar", "sensor_msgs/msg/PointCloud2", sensor_qos},
        {"/right/hesai/pandar", "sensor_msgs/msg/PointCloud2", sensor_qos},
    };

    std::vector<TopicConfig> configs;
    configs.reserve(prefixed.size() + globals.size());

    const std::string prefix = "/" + unit_name + "/";
    for (const auto & entry : prefixed) {
        TopicConfig config;
        config.topic = prefix + std::get<0>(entry);
        config.type_string = std::get<1>(entry);
        config.qos = std::get<2>(entry);
        configs.emplace_back(std::move(config));
    }
    for (const auto & entry : globals) {
        TopicConfig config;
        config.topic = std::get<0>(entry);
        config.type_string = std::get<1>(entry);
        config.qos = std::get<2>(entry);
        configs.emplace_back(std::move(config));
    }

    return configs;
}

class RerunBridge : public rclcpp::Node {
public:
    RerunBridge(std::string unit_name, rerun::RecordingStream rec)
        : rclcpp::Node("p13_rerun_bridge"), unit_name_(std::move(unit_name)), rec_(std::move(rec)) {
        initialize_handlers();
        const auto configs = build_topic_list(unit_name_);
        for (const auto & config : configs) {
            try {
                create_subscription_for(config);
            } catch (const std::exception & err) {
                RCLCPP_ERROR(get_logger(), "Failed to subscribe %s (%s): %s", config.topic.c_str(), config.type_string.c_str(), err.what());
            }
        }
    }

private:
    using Handler = std::function<void(const std::string &, const std::string &, const MessageMembers &, const void *)>;

    void initialize_handlers() {
        handlers_["geometry_msgs/msg/Twist"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_twist(rec_, unit, topic, *reinterpret_cast<const geometry_msgs::msg::Twist *>(data));
        };
        handlers_["geometry_msgs/msg/Pose"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_pose_message(rec_, unit, topic, *reinterpret_cast<const geometry_msgs::msg::Pose *>(data));
        };
        handlers_["geometry_msgs/msg/PoseStamped"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_pose_stamped(rec_, unit, topic, *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(data));
        };
        handlers_["geometry_msgs/msg/PointStamped"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_point_stamped(rec_, unit, topic, *reinterpret_cast<const geometry_msgs::msg::PointStamped *>(data));
        };
        handlers_["nav_msgs/msg/Odometry"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_odometry(rec_, unit, topic, *reinterpret_cast<const nav_msgs::msg::Odometry *>(data));
        };
        handlers_["nav_msgs/msg/Path"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_path(rec_, unit, topic, *reinterpret_cast<const nav_msgs::msg::Path *>(data));
        };
        handlers_["std_msgs/msg/String"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_string(rec_, unit, topic, *reinterpret_cast<const std_msgs::msg::String *>(data));
        };
        handlers_["std_msgs/msg/UInt8"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            const auto * msg = reinterpret_cast<const std_msgs::msg::UInt8 *>(data);
            handle_uint_scalar(rec_, unit, topic, static_cast<std::uint64_t>(msg->data));
        };
        handlers_["std_msgs/msg/UInt16"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            const auto * msg = reinterpret_cast<const std_msgs::msg::UInt16 *>(data);
            handle_uint_scalar(rec_, unit, topic, static_cast<std::uint64_t>(msg->data));
        };
        handlers_["std_msgs/msg/UInt64"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            const auto * msg = reinterpret_cast<const std_msgs::msg::UInt64 *>(data);
            handle_uint_scalar(rec_, unit, topic, msg->data);
        };
        handlers_["std_msgs/msg/Float64MultiArray"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_float64_multi_array(rec_, unit, topic, *reinterpret_cast<const std_msgs::msg::Float64MultiArray *>(data));
        };
        handlers_["sensor_msgs/msg/Imu"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_imu(rec_, unit, topic, *reinterpret_cast<const sensor_msgs::msg::Imu *>(data));
        };
        handlers_["sensor_msgs/msg/PointCloud2"] = [this](const std::string & unit, const std::string & topic, const MessageMembers &, const void * data) {
            handle_point_cloud(rec_, unit, topic, *reinterpret_cast<const sensor_msgs::msg::PointCloud2 *>(data));
        };
    }

    void create_subscription_for(const TopicConfig & config) {
        auto context = std::make_shared<SubscriptionContext>();
        context->members = load_members(config.type_string, context->type_library, context->type_support);

        auto qos = config.qos.value_or(rclcpp::QoS(10));
        auto callback = [this, context, topic = config.topic, type_string = config.type_string](std::shared_ptr<rclcpp::SerializedMessage> serialized) {
            auto storage = std::unique_ptr<uint8_t[]>(new uint8_t[context->members->size_of_]);
            context->members->init_function(storage.get(), rosidl_runtime_cpp::MessageInitialization::ALL);

            auto ret = rmw_deserialize(serialized->get_rcl_serialized_message(), context->type_support, storage.get());
            if (ret != RMW_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message on %s", topic.c_str());
                context->members->fini_function(storage.get());
                return;
            }

            if (auto stamp = extract_header_stamp(*context->members, storage.get())) {
                rec_.set_time_seconds("ros_time", *stamp);
            }

            const auto handler_it = handlers_.find(type_string);
            if (handler_it != handlers_.end()) {
                handler_it->second(unit_name_, topic, *context->members, storage.get());
            } else {
                handle_as_scalars(rec_, unit_name_, topic, *context->members, storage.get());
            }

            context->members->fini_function(storage.get());
        };

        context->subscription = this->create_generic_subscription(
            config.topic,
            config.type_string,
            qos,
            callback);

        if (!context->subscription) {
            throw std::runtime_error("create_generic_subscription returned null");
        }

        subscriptions_.push_back(std::move(context));
        RCLCPP_INFO(get_logger(), "Subscribed to %s (%s)", config.topic.c_str(), config.type_string.c_str());
    }

    std::string unit_name_;
    rerun::RecordingStream rec_;
    std::unordered_map<std::string, Handler> handlers_;
    std::vector<std::shared_ptr<SubscriptionContext>> subscriptions_;
};

rerun::RecordingStream initialize_rerun(const std::string & unit) {
    rerun::RecordingStream rec(unit + "_ros_bridge");
    const auto mode_env = std::getenv("RERUN_MODE");
    const std::string mode = mode_env ? std::string(mode_env) : std::string("connect");
    if (mode == "spawn") {
        auto res = rec.spawn();
        if (res.is_err()) {
            std::cerr << "Failed to spawn Rerun viewer: " << res.message() << '\n';
        }
    } else {
        const char * server_env = std::getenv("RERUN_SERVER");
        const std::string server = server_env ? std::string(server_env) : std::string("127.0.0.1:9876");
        auto res = rec.connect(server);
        if (res.is_err()) {
            std::cerr << "Failed to connect to Rerun server at " << server << ": " << res.message() << '\n';
        }
    }
    return rec;
}

}  // namespace

int main(int argc, char ** argv) {
    try {
        const char * env_unit = std::getenv("P13_NAME");
        std::string unit = normalize_unit(env_unit ? std::string(env_unit) : std::string("P13"));
        auto rec = initialize_rerun(unit);

        rclcpp::init(argc, argv);
        auto node = std::make_shared<RerunBridge>(unit, std::move(rec));
        rclcpp::spin(node);
        rclcpp::shutdown();
        return EXIT_SUCCESS;
    } catch (const std::exception & err) {
        std::cerr << "p13_rerun_bridge encountered an error: " << err.what() << '\n';
        return EXIT_FAILURE;
    }
}

