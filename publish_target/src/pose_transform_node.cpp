#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <optional>
#include <thread>
#include "bumblebee_interfaces/msg/object_data.hpp"
#include <std_msgs/msg/bool.hpp>
#include <set>  
#include <bumblebee_interfaces/msg/manipulator_target.hpp>

const float POSITION_EPSILON = 0.09f; // 위치 변화 허용 범위 (예: 3cm)
const double VELOCITY_CHANGE_THRESHOLD = 0.002;  // 예:

class PoseTransformer : public rclcpp::Node
{
public:
  PoseTransformer()
  : Node("pose_transformer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    input_sub_ = this->create_subscription<bumblebee_interfaces::msg::ObjectData>(
    "/object_data", 10,
    std::bind(&PoseTransformer::poseCallback, this, std::placeholders::_1));

    manipulator_busy_ = this->create_subscription<std_msgs::msg::Bool>(
    "/manipulator_busy", 10,
    std::bind(&PoseTransformer::maniCallback, this, std::placeholders::_1));

    manipulator_target_pub_ = this->create_publisher<bumblebee_interfaces::msg::ManipulatorTarget>("/tras_base", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PoseTransformer::jointStateCallback, this, std::placeholders::_1));
  }
private:
  std::map<int, geometry_msgs::msg::PoseStamped> object_map_;
  std::set<int> published_ids_;  // 이미 퍼블리시된 ID 추적
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<bumblebee_interfaces::msg::ManipulatorTarget>::SharedPtr manipulator_target_pub_;
  rclcpp::Subscription<bumblebee_interfaces::msg::ObjectData>::SharedPtr input_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manipulator_busy_;
  std::map<int, geometry_msgs::msg::PoseStamped> base_footprint_pose_map_;
  bool mani_busy;
  double left_vel = 0.0;
  double right_vel = 0.3;
  int current_id_ = 1;
  rclcpp::Time last_motion_time_ = this->now();  // 시작 시간


  // 안정적인 TF 변환 함수
  std::optional<geometry_msgs::msg::PoseStamped> transformCameraToMap(const geometry_msgs::msg::PoseStamped& camera_pose)
  {
    const std::string source_frame = "camera_color_optical_frame";
    const std::string target_frame = "map";

    if (!tf_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero)) {
      for (int i = 0; i < 10; ++i) {
        RCLCPP_WARN(this->get_logger(), "⏳ Waiting for TF: %s → %s", source_frame.c_str(), target_frame.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (tf_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero)) {
          break;
        }
      }
      if (!tf_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero)) {
        RCLCPP_ERROR(this->get_logger(), "❌ Transform not available: %s → %s", source_frame.c_str(), target_frame.c_str());
        return std::nullopt;
      }
    }

    try {
      auto transform = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped map_pose;
      tf2::doTransform(camera_pose, map_pose, transform);
      return map_pose;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "⚠ TF Error: %s", ex.what());
      return std::nullopt;
    }
  }

  void maniCallback(const std_msgs::msg::Bool msg){
    mani_busy = msg.data;
  }

  bool is_duplicate(const geometry_msgs::msg::Pose& new_pose, int& existing_id) {
  for (const auto& [id, obj_pose_stamped] : object_map_) {
    const auto& obj_pose = obj_pose_stamped.pose;  // ⬅️ 여기서 pose 추출
    float dx = obj_pose.position.x - new_pose.position.x;
    float dy = obj_pose.position.y - new_pose.position.y;
    float dz = obj_pose.position.z - new_pose.position.z;
    float dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq < POSITION_EPSILON * POSITION_EPSILON) {
      existing_id = id;
      return true;
    }
  }
  return false;
}

void poseCallback(const bumblebee_interfaces::msg::ObjectData::SharedPtr msg)
{
  if (!mani_busy) {
    RCLCPP_INFO(this->get_logger(), "📥 poseCallback 진입: name=%s, x=%.2f, y=%.2f, z=%.2f",
                msg->name.c_str(), msg->x, msg->y, msg->z);
    
    geometry_msgs::msg::PoseStamped pose_in_camera;
    pose_in_camera.header.stamp = this->now();
    pose_in_camera.header.frame_id = "camera_color_optical_frame";
    pose_in_camera.pose.position.x = msg->x;
    pose_in_camera.pose.position.y = msg->y;
    pose_in_camera.pose.position.z = msg->z;
    pose_in_camera.pose.orientation.w = 1.0;


    auto maybe_map_pose = transformCameraToMap(pose_in_camera);
    if (!maybe_map_pose) {
      RCLCPP_WARN(this->get_logger(), "⚠️ map 변환 실패, 무시됨");
      return;
    }

    int id_to_use;
    if (is_duplicate(maybe_map_pose->pose, id_to_use)) {
      object_map_[id_to_use] = *maybe_map_pose;
      RCLCPP_INFO(this->get_logger(), "♻️ 기존 객체 ID %d 위치 갱신됨 (map 기준): x=%.2f y=%.2f z=%.2f",
                  id_to_use,
                  maybe_map_pose->pose.position.x,
                  maybe_map_pose->pose.position.y,
                  maybe_map_pose->pose.position.z);
    } else {
      id_to_use = current_id_;
      object_map_[id_to_use] = *maybe_map_pose;
      RCLCPP_INFO(this->get_logger(), "🌸 새로운 객체 ID %d 등록됨 (map 기준): x=%.2f y=%.2f z=%.2f",
                  id_to_use,
                  maybe_map_pose->pose.position.x,
                  maybe_map_pose->pose.position.y,
                  maybe_map_pose->pose.position.z);
      current_id_++;
    }

    // ✅ base_footprint 기준으로 변환해서 저장
      if (checkRobotStopped(3.0)) {
    try {
      auto transform = tf_buffer_.lookupTransform("base_footprint", "map", tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped pose_in_base;
      tf2::doTransform(*maybe_map_pose, pose_in_base, transform);
      base_footprint_pose_map_[id_to_use] = pose_in_base;
      RCLCPP_INFO(this->get_logger(), "📌 base_footprint 기준 좌표 저장됨: ID=%d", id_to_use);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "❌ TF 변환 실패 (base_footprint): %s", ex.what());
    }

      }
  }
  
}

bool checkRobotStopped(double duration_sec = 2.0)
{
  rclcpp::Time now = this->now();
  rclcpp::Duration duration_since_motion = now - last_motion_time_;
  return duration_since_motion.seconds() >= duration_sec;
  
}

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
   for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "wheel_left_joint")  left_vel = msg->velocity[i];
    if (msg->name[i] == "wheel_right_joint") right_vel = msg->velocity[i];
  }


  if (left_vel != 0.0 || right_vel != 0.0) {
  last_motion_time_ = this->now();  // 🟢 로봇이 움직이면 시간 초기화
  }

  if(!mani_busy){
  

  if (checkRobotStopped(3.0)) {
    
    
    // 🌟 z값 기준으로 정렬을 위한 벡터 복사
    std::vector<std::pair<int, geometry_msgs::msg::PoseStamped>> sorted_targets(
        base_footprint_pose_map_.begin(), base_footprint_pose_map_.end());

    // 🌟 z 오름차순 정렬
    std::sort(sorted_targets.begin(), sorted_targets.end(),
              [](const auto& a, const auto& b) {
                return a.second.pose.position.z < b.second.pose.position.z;
              });

    // 🌟 정렬된 순서대로 퍼블리시
    for (const auto& [id, pose_in_base] : sorted_targets) {
      bumblebee_interfaces::msg::ManipulatorTarget msg;
      msg.id = id;
      msg.fertilized = false;
      msg.pose = pose_in_base;
      msg.header.stamp = this->now();
      msg.header.frame_id = "base_footprint";

      manipulator_target_pub_->publish(msg);

      RCLCPP_INFO(this->get_logger(), "📤 ID %d 퍼블리시됨 (z=%.2f): x=%.2f y=%.2f",
                  id,
                  pose_in_base.pose.position.z,
                  pose_in_base.pose.position.x,
                  pose_in_base.pose.position.y);
    }
  }
 
  }
  else
  RCLCPP_INFO(this->get_logger(), "⛔ 매니퓰레이터 동작 중 jointcallback 부분 - 객체 무시됨");
}



};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
/*
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <optional>
#include <thread>
#include "bumblebee_interfaces/msg/object_data.hpp"
#include <std_msgs/msg/bool.hpp>
#include <set>  // 추가
#include <bumblebee_interfaces/msg/manipulator_target.hpp>

const float POSITION_EPSILON = 0.07f; // 위치 변화 허용 범위 (예: 3cm)

class PoseTransformer : public rclcpp::Node
{
public:
  PoseTransformer()
  : Node("pose_transformer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    input_sub_ = this->create_subscription<bumblebee_interfaces::msg::ObjectData>(
    "/object_data", 10,
    std::bind(&PoseTransformer::poseCallback, this, std::placeholders::_1));

    manipulator_busy_ = this->create_subscription<std_msgs::msg::Bool>(
    "/manipulator_busy", 10,
    std::bind(&PoseTransformer::maniCallback, this, std::placeholders::_1));

    manipulator_target_pub_ = this->create_publisher<bumblebee_interfaces::msg::ManipulatorTarget>("/tras_base", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PoseTransformer::jointStateCallback, this, std::placeholders::_1));
  }

private:
  std::map<int, geometry_msgs::msg::PoseStamped> object_map_;
  std::set<int> published_ids_;  // 이미 퍼블리시된 ID 추적
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<bumblebee_interfaces::msg::ManipulatorTarget>::SharedPtr manipulator_target_pub_;
  rclcpp::Subscription<bumblebee_interfaces::msg::ObjectData>::SharedPtr input_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manipulator_busy_;
  std::map<int, geometry_msgs::msg::PoseStamped> base_footprint_pose_map_;
  bool mani_busy;
  double left_vel = 0.0;
  double right_vel = 0.3;
  int flag = 0;
  int current_id_ = 1;

  // 안정적인 TF 변환 함수
  std::optional<geometry_msgs::msg::PoseStamped> transformCameraToMap(const geometry_msgs::msg::PoseStamped& camera_pose)
  {
    const std::string source_frame = "camera_color_optical_frame";
    const std::string target_frame = "map";

    if (!tf_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero)) {
      for (int i = 0; i < 10; ++i) {
        RCLCPP_WARN(this->get_logger(), "⏳ Waiting for TF: %s → %s", source_frame.c_str(), target_frame.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (tf_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero)) {
          break;
        }
      }
      if (!tf_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero)) {
        RCLCPP_ERROR(this->get_logger(), "❌ Transform not available: %s → %s", source_frame.c_str(), target_frame.c_str());
        return std::nullopt;
      }
    }

    try {
      auto transform = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped map_pose;
      tf2::doTransform(camera_pose, map_pose, transform);
      return map_pose;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "⚠ TF Error: %s", ex.what());
      return std::nullopt;
    }
  }

  void maniCallback(const std_msgs::msg::Bool msg){
    mani_busy = msg.data;
  }

  bool is_duplicate(const geometry_msgs::msg::Pose& new_pose, int& existing_id) {
  for (const auto& [id, obj_pose_stamped] : object_map_) {
    const auto& obj_pose = obj_pose_stamped.pose;  // ⬅️ 여기서 pose 추출
    float dx = obj_pose.position.x - new_pose.position.x;
    float dy = obj_pose.position.y - new_pose.position.y;
    float dz = obj_pose.position.z - new_pose.position.z;
    float dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq < POSITION_EPSILON * POSITION_EPSILON) {
      existing_id = id;
      return true;
    }
  }
  return false;
}

void poseCallback(const bumblebee_interfaces::msg::ObjectData::SharedPtr msg)
{
  
  if(left_vel == 0.0 && right_vel == 0.0){
  if (!mani_busy) {
    RCLCPP_INFO(this->get_logger(), "📥 poseCallback 진입: name=%s, x=%.2f, y=%.2f, z=%.2f",
                msg->name.c_str(), msg->x, msg->y, msg->z);
    
    if(flag == 0)
    std::this_thread::sleep_for(std::chrono::seconds(2));
    geometry_msgs::msg::PoseStamped pose_in_camera;
    pose_in_camera.header.stamp = this->now();
    pose_in_camera.header.frame_id = "camera_color_optical_frame";
    pose_in_camera.pose.position.x = msg->x;
    pose_in_camera.pose.position.y = msg->y;
    pose_in_camera.pose.position.z = msg->z;
    pose_in_camera.pose.orientation.w = 1.0;

    flag = flag + 1;

    auto maybe_map_pose = transformCameraToMap(pose_in_camera);
    if (!maybe_map_pose) {
      RCLCPP_WARN(this->get_logger(), "⚠️ map 변환 실패, 무시됨");
      return;
    }

    int id_to_use;
    if (is_duplicate(maybe_map_pose->pose, id_to_use)) {
      object_map_[id_to_use] = *maybe_map_pose;
      RCLCPP_INFO(this->get_logger(), "♻️ 기존 객체 ID %d 위치 갱신됨 (map 기준): x=%.2f y=%.2f z=%.2f",
                  id_to_use,
                  maybe_map_pose->pose.position.x,
                  maybe_map_pose->pose.position.y,
                  maybe_map_pose->pose.position.z);
    } else {
      id_to_use = current_id_;
      object_map_[id_to_use] = *maybe_map_pose;
      RCLCPP_INFO(this->get_logger(), "🌸 새로운 객체 ID %d 등록됨 (map 기준): x=%.2f y=%.2f z=%.2f",
                  id_to_use,
                  maybe_map_pose->pose.position.x,
                  maybe_map_pose->pose.position.y,
                  maybe_map_pose->pose.position.z);
      current_id_++;
    }

    // ✅ base_footprint 기준으로 변환해서 저장
    try {
      auto transform = tf_buffer_.lookupTransform("base_footprint", "map", tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped pose_in_base;
      tf2::doTransform(*maybe_map_pose, pose_in_base, transform);
      base_footprint_pose_map_[id_to_use] = pose_in_base;
      RCLCPP_INFO(this->get_logger(), "📌 base_footprint 기준 좌표 저장됨: ID=%d", id_to_use);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "❌ TF 변환 실패 (base_footprint): %s", ex.what());
    }

  } else {
    RCLCPP_INFO(this->get_logger(), "⛔ 매니퓰레이터 동작 중 - 객체 무시됨");
  }
  }
}


void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if(!mani_busy){
  
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "wheel_left_joint")  left_vel = msg->velocity[i];
    if (msg->name[i] == "wheel_right_joint") right_vel = msg->velocity[i];
  }

  if (left_vel == 0.0 && right_vel == 0.0 && !base_footprint_pose_map_.empty()) {
    
    // 🌟 z값 기준으로 정렬을 위한 벡터 복사
    std::vector<std::pair<int, geometry_msgs::msg::PoseStamped>> sorted_targets(
        base_footprint_pose_map_.begin(), base_footprint_pose_map_.end());

    // 🌟 z 오름차순 정렬
    std::sort(sorted_targets.begin(), sorted_targets.end(),
              [](const auto& a, const auto& b) {
                return a.second.pose.position.z < b.second.pose.position.z;
              });

    // 🌟 정렬된 순서대로 퍼블리시
    for (const auto& [id, pose_in_base] : sorted_targets) {
      bumblebee_interfaces::msg::ManipulatorTarget msg;
      msg.id = id;
      msg.fertilized = false;
      msg.pose = pose_in_base;
      msg.header.stamp = this->now();
      msg.header.frame_id = "base_footprint";

      manipulator_target_pub_->publish(msg);

      RCLCPP_INFO(this->get_logger(), "📤 ID %d 퍼블리시됨 (z=%.2f): x=%.2f y=%.2f",
                  id,
                  pose_in_base.pose.position.z,
                  pose_in_base.pose.position.x,
                  pose_in_base.pose.position.y);
    }
  }
  }
  else
  RCLCPP_INFO(this->get_logger(), "⛔ 매니퓰레이터 동작 중 jointcallback 부분 - 객체 무시됨");
}



};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/