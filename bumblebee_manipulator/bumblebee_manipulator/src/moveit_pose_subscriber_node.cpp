/*#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <bumblebee_interfaces/msg/manipulator_target.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cmath>
#include <map>
#include <thread>  
#include "bumblebee_interfaces/msg/lift_control.hpp"
#include <std_msgs/msg/bool.hpp>

#define WORKSPACE_LIMIT_Z 105

constexpr float POSITION_EPSILON = 0.05101560f;
const std::string PLANNING_GROUP = "arm";

struct TARGET_INFO {
  int id = -1;
  bool fertilized = false;
  float x;
  float y;
  float z;
  rclcpp::Time last_updated; 
};



class MoveGroupInterfaceNode {
public:
  MoveGroupInterfaceNode(const rclcpp::Node::SharedPtr& node) 
  : node_(node),
    move_group_(node_, PLANNING_GROUP),
    robot_model_loader_(node_, "robot_description"),
    kinematic_model_(robot_model_loader_.getModel())
  {
    move_group_.setEndEffectorLink("real_hand");
    move_group_.setPoseReferenceFrame("manipulator_joint_base_1");
//manipulator_joint_base_1

    pose_stamped_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/manipulator_target_pose", 10, std::bind(&MoveGroupInterfaceNode::poseStampedCallback, this, std::placeholders::_1));


    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MoveGroupInterfaceNode::jointStateCallback, this, std::placeholders::_1));


    //lift_control_sub_ = node_->create_subscription<bumblebee_interfaces::msg::LiftControl>(
      //"/lift_control_topic", 10, std::bind(&MoveGroupInterfaceNode::liftControlCallback, this, std::placeholders::_1));

    auto_exec_timer_ = node_->create_wall_timer(
      std::chrono::seconds(5), std::bind(&MoveGroupInterfaceNode::processUnfertilizedTargets, this));

    lift_pub_ = node_->create_publisher<bumblebee_interfaces::msg::LiftControl>("lift_control_topic", 10);

    busy_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/manipulator_busy", 10);

    RCLCPP_INFO(node_->get_logger(), "MoveGroupInterfaceNode is ready!");
    
  }

  
  void printTargetMap() const {
    RCLCPP_INFO(node_->get_logger(), "📦 Target Map 상태:");
    for (const auto& [id, obj] : target_map_) {
      RCLCPP_INFO(node_->get_logger(),
        "ID: %d | fertilized: %s | pos: (%.7f, %.7f, %.7f)",
        id,
        obj.fertilized ? "true" : "false",
        obj.x, obj.y, obj.z
      );
    }
  }
  
  const sensor_msgs::msg::JointState& getLastJointState() const {
    return last_joint_state_;
  }

 
  void perform_pollination_with_joint7(const std::vector<double>& base_joint_positions, double delta_joint7) {
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_.getRobotModel()->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
    std::vector<double> joint_positions = base_joint_positions;
  
    // joint7 인덱스 찾기
    size_t idx = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), "joint7"));
    if (idx >= joint_positions.size()) {
      RCLCPP_ERROR(node_->get_logger(), "❌ joint7 인덱스를 찾을 수 없습니다.");
      return;
    }
  
    // 🔁 +delta 이동
    joint_positions[idx] += delta_joint7;
    move_group_.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    if (move_group_.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_.execute(plan1);
      RCLCPP_INFO(node_->get_logger(), "🌀 joint7 +delta 수행");
    }
  
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
    // 🔁 -delta 복귀
    joint_positions[idx] -= delta_joint7;
    move_group_.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    if (move_group_.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_.execute(plan2);
      RCLCPP_INFO(node_->get_logger(), "🌀 joint7 -delta 복귀");
    }
  
    move_group_.clearPoseTargets();
  }
  
  void search_JointState1() {

    std_msgs::msg::Bool busy_msg;
    busy_msg.data = true;
    busy_pub_->publish(busy_msg);
    std::map<std::string, double> joint_values = {
      {"joint1", 0.00613892},
      {"joint2", 0.78559000},
      {"joint3", -1.54000001},
      {"joint4", -0.00153398},
      {"joint5", 0.0210000},
      {"joint6", -0.00613592},
      {"joint7", 0.00000000}
    };
  
    move_group_.setJointValueTarget(joint_values);
  
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "📌 고정 Joint 값으로 플래닝 성공!");
      if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "✅ 고정 자세 실행 완료!");
      } else {
        RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 실행 실패");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 플래닝 실패");
    }
  
    move_group_.clearPoseTargets();
    busy_msg.data = false;
    busy_pub_->publish(busy_msg);
  }


  void search_JointState() {
    std::map<std::string, double> joint_values = {
      {"joint1", 0.00613892},
      {"joint2", 0.78559000},
      {"joint3", -1.54000001},
      {"joint4", -0.00153398},
      {"joint5", 0.0210000},
      {"joint6", -0.00613592},
      {"joint7", 0.00000000}
    };
  
    move_group_.setJointValueTarget(joint_values);
  
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "📌 고정 Joint 값으로 플래닝 성공!");
      if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "✅ 고정 자세 실행 완료!");
      } else {
        RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 실행 실패");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 플래닝 실패");
    }
  
    move_group_.clearPoseTargets();
  }
  
private:
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // 🔎 중복 좌표 검사 (기존에 비슷한 위치에 이미 등록된 객체 있는지 확인)
  for (const auto& [existing_id, obj] : target_map_) {
    float dx = obj.x - msg->pose.position.x;
    float dy = obj.y - msg->pose.position.y;
    float dz = obj.z - msg->pose.position.z;
    float dist_sq = dx * dx + dy * dy + dz * dz;

    if (dist_sq < POSITION_EPSILON * POSITION_EPSILON) {
      RCLCPP_INFO(node_->get_logger(), "⚠️ 비슷한 좌표 이미 존재. 등록 생략");
      return;
    }
  }

  // ✅ 새로운 객체 등록
  int id = next_id_++;

  TARGET_INFO info;
  info.id = id;
  info.fertilized = false;
  info.x = msg->pose.position.x;
  info.y = msg->pose.position.y;
  info.z = msg->pose.position.z;
  info.last_updated = node_->now();

  target_map_[id] = info;
  RCLCPP_INFO(node_->get_logger(), "🆕 PoseStamped 기반 타겟 ID %d 등록됨 (x=%.2f y=%.2f z=%.2f)", id, info.x, info.y, info.z);
}



  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_joint_state_ = *msg;
  }
  
  void check_target(){
    for (auto& [id, obj] : target_map_) {
    if(obj.fertilized) continue;
    else{
      obj.z -= 0.069f;  // Z값 감소
      RCLCPP_INFO(node_->get_logger(), "ID %d의 Z값 감소: %.3f", id, obj.z);
      bumblebee_interfaces::msg::LiftControl lift_msg;
      lift_msg.flag = 1;
      lift_msg.value = 105.0f;
      lift_pub_->publish(lift_msg);
      std::this_thread::sleep_for(std::chrono::seconds(6));
      printTargetMap();
      manipulation();
        lift_msg.flag = 1;
                  lift_msg.value = 104.0f;
                  lift_pub_ -> publish(lift_msg);
                  std::this_thread::sleep_for(std::chrono::seconds(15));
      break;
    }
    }

  }

  void manipulation(){
    RCLCPP_WARN(node_->get_logger(), "실행 발생!");
  
  for (auto& [id, obj] : target_map_) {
    if (obj.fertilized || (obj.z >1.05)) continue;

    if (last_joint_state_.name.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Joint state not received. Skipping ID %d", id);
      continue;
    }

    std_msgs::msg::Bool busy_msg;
    busy_msg.data = true;
    busy_pub_->publish(busy_msg);


    // IK, planning, execution ... (이하 기존 로직 동일)


    // ✅ RobotState 생성 및 IK 기반 자세 계산
    moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(kinematic_model_));
    current_state->setToDefaultValues();

    const moveit::core::JointModelGroup* joint_model_group =
        kinematic_model_->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::msg::Pose pose;
    pose.position.x = obj.x;
    pose.position.y = obj.y;
    pose.position.z = obj.z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    std::vector<double> solution;
    bool found_ik = false;

    if (current_state->setFromIK(joint_model_group, pose, move_group_.getEndEffectorLink(), 0.1)) {
      current_state->copyJointGroupPositions(joint_model_group, solution);
      found_ik = true;
    }

    if (!found_ik) {
      RCLCPP_WARN(node_->get_logger(), "❌ IK 해를 찾을 수 없음. ID %d", id);
      continue;
    }

    // ✅ 플래닝 및 실행
    move_group_.setJointValueTarget(solution);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        obj.fertilized = true;
        RCLCPP_INFO(node_->get_logger(), "✅ 수분 위치 도달 ID %d", id);
      } else {
        RCLCPP_WARN(node_->get_logger(), "❌ 실행 실패 ID %d", id);
        continue;
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "❌ 계획 실패 ID %d", id);
      continue;
    }

    move_group_.clearPoseTargets();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ✅ 수분 동작 수행
    perform_pollination_with_joint7(solution, 0.177266);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    perform_pollination_with_joint7(solution, -0.177266);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ✅ 초기 자세 복귀
    search_JointState();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    busy_msg.data = false;
    busy_pub_->publish(busy_msg);
  }
  check_target();
  }

  void processUnfertilizedTargets() {
    
    manipulation();
}



  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;

  rclcpp::Subscription<bumblebee_interfaces::msg::ManipulatorTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr auto_exec_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr busy_pub_;
  rclcpp::Publisher<bumblebee_interfaces::msg::LiftControl>::SharedPtr lift_pub_;

  sensor_msgs::msg::JointState last_joint_state_;
  std::map<int, TARGET_INFO> target_map_;
  int next_id_ = 1;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_auto_executor_node");
  MoveGroupInterfaceNode move_group_interface_node(node);
  move_group_interface_node.search_JointState1();
  rclcpp::spin(node);
  rclcpp::shutdown();
  move_group_interface_node.printTargetMap();
   
   RCLCPP_INFO(node->get_logger(), "💡 프로그램 종료. 최종 Joint 값 출력:");
   const auto& joint_state = move_group_interface_node.getLastJointState();
   for (size_t i = 0; i < joint_state.name.size(); ++i) {
     RCLCPP_INFO(node->get_logger(), "🦿 %s = %.8f", joint_state.name[i].c_str(), joint_state.position[i]);
   }
  return 0;
}
*/


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <bumblebee_interfaces/msg/manipulator_target.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cmath>
#include <map>
#include <thread>  
#include "bumblebee_interfaces/msg/lift_control.hpp"
#include <std_msgs/msg/bool.hpp>
#define WORKSPACE_LIMIT_Z 105

constexpr float POSITION_EPSILON = 0.05101560f;
const std::string PLANNING_GROUP = "arm";

struct TARGET_INFO {
  int id = -1;
  bool fertilized = false;
  float x;
  float y;
  float z;
  rclcpp::Time last_updated; 
};



class MoveGroupInterfaceNode {
public:
  MoveGroupInterfaceNode(const rclcpp::Node::SharedPtr& node) 
  : node_(node),
    move_group_(node_, PLANNING_GROUP),
    robot_model_loader_(node_, "robot_description"),
    kinematic_model_(robot_model_loader_.getModel())
  {
    move_group_.setEndEffectorLink("real_hand");
    move_group_.setPoseReferenceFrame("manipulator_joint_base_1");
//manipulator_joint_base_1
    //publisher_ = this->create_publisher<std_msgs::msg::Bool>("bool_flag", 10); // manipulator 실행 중 알려주는 변수

    target_sub_ = node_->create_subscription<bumblebee_interfaces::msg::ManipulatorTarget>(
      "/tras_base", 10, std::bind(&MoveGroupInterfaceNode::targetCallback, this, std::placeholders::_1));

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MoveGroupInterfaceNode::jointStateCallback, this, std::placeholders::_1));


    //lift_control_sub_ = node_->create_subscription<bumblebee_interfaces::msg::LiftControl>(
      //"/lift_control_topic", 10, std::bind(&MoveGroupInterfaceNode::liftControlCallback, this, std::placeholders::_1));

    auto_exec_timer_ = node_->create_wall_timer(
      std::chrono::seconds(5), std::bind(&MoveGroupInterfaceNode::processUnfertilizedTargets, this));

    lift_pub_ = node_->create_publisher<bumblebee_interfaces::msg::LiftControl>("lift_control_topic", 10);

  
    busy_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/manipulator_busy", 10);

    RCLCPP_INFO(node_->get_logger(), "MoveGroupInterfaceNode is ready!");
    
  }

  
  void printTargetMap() const {
    RCLCPP_INFO(node_->get_logger(), "📦 Target Map 상태:");
    for (const auto& [id, obj] : target_map_) {
      RCLCPP_INFO(node_->get_logger(),
        "ID: %d | fertilized: %s | pos: (%.7f, %.7f, %.7f)",
        id,
        obj.fertilized ? "true" : "false",
        obj.x, obj.y, obj.z
      );
    }
  }
  
  const sensor_msgs::msg::JointState& getLastJointState() const {
    return last_joint_state_;
  }

 
  void perform_pollination_with_joint7(const std::vector<double>& base_joint_positions, double delta_joint7) {
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_.getRobotModel()->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
    std::vector<double> joint_positions = base_joint_positions;
  
    // joint7 인덱스 찾기
    size_t idx = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), "joint7"));
    if (idx >= joint_positions.size()) {
      RCLCPP_ERROR(node_->get_logger(), "❌ joint7 인덱스를 찾을 수 없습니다.");
      return;
    }
  
    // 🔁 +delta 이동
    joint_positions[idx] += delta_joint7;
    move_group_.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    if (move_group_.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_.execute(plan1);
      RCLCPP_INFO(node_->get_logger(), "🌀 joint7 +delta 수행");
    }
  
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
    // 🔁 -delta 복귀
    joint_positions[idx] -= delta_joint7;
    move_group_.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    if (move_group_.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_.execute(plan2);
      RCLCPP_INFO(node_->get_logger(), "🌀 joint7 -delta 복귀");
    }
  
    move_group_.clearPoseTargets();
  }
  
  void search_JointState1() {

    std_msgs::msg::Bool busy_msg;
    busy_msg.data = true;
    busy_pub_->publish(busy_msg);
    std::map<std::string, double> joint_values = {
      {"joint1", 0.00613892},
      {"joint2", 0.78559000},
      {"joint3", -1.54000001},
      {"joint4", -0.00153398},
      {"joint5", 0.0210000},
      {"joint6", -0.00613592},
      {"joint7", 0.00000000}
    };
  
    move_group_.setJointValueTarget(joint_values);
  
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "📌 고정 Joint 값으로 플래닝 성공!");
      if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "✅ 고정 자세 실행 완료!");
      } else {
        RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 실행 실패");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 플래닝 실패");
    }
  
    move_group_.clearPoseTargets();
      std::this_thread::sleep_for(std::chrono::seconds(2));
    busy_msg.data = false;
    busy_pub_->publish(busy_msg);
  }

  void refill_flower(){
    std::map<std::string, double> joint_values = { // 140, -22, -97, -0, -21, 0, 0
      {"joint1", 2.443460953},
      {"joint2", -0.38397243},
      {"joint3", -1.69296937},
      {"joint4", 0.0000000},
      {"joint5", -0.366519143},
      {"joint6", 0.0000000},
      {"joint7", 0.00000000}
    };
  
    move_group_.setJointValueTarget(joint_values);
  
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "꽃가루 찾으러 가자!");
      if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "✅ 고정 자세 실행 완료!");
      } else {
        RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 실행 실패");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 플래닝 실패");
    }
  
    move_group_.clearPoseTargets();
  }

  void search_JointState() {
    std::map<std::string, double> joint_values = {
      {"joint1", 0.00613892},
      {"joint2", 0.78559000},
      {"joint3", -1.54000001},
      {"joint4", -0.00153398},
      {"joint5", 0.0210000},
      {"joint6", -0.00613592},
      {"joint7", 0.00000000}
    };
  
    move_group_.setJointValueTarget(joint_values);
  
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "📌 고정 Joint 값으로 플래닝 성공!");
      if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "✅ 고정 자세 실행 완료!");
      } else {
        RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 실행 실패");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "⚠ 고정 자세 플래닝 실패");
    }
  
    move_group_.clearPoseTargets();
  }
  
private:
  void targetCallback(const bumblebee_interfaces::msg::ManipulatorTarget::SharedPtr msg) {
    
    if (last_joint_state_.name.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Joint state not yet received. Skipping target.");
      return;
    }
    
    for (size_t i = 0; i < last_joint_state_.name.size(); ++i) {
      if (last_joint_state_.name[i] == "wheel_left_joint") left_vel = last_joint_state_.velocity[i];
      if (last_joint_state_.name[i] == "wheel_right_joint") right_vel = last_joint_state_.velocity[i];
    }
    if(left_vel != 0.0 || right_vel != 0.0)
      return;

    int id = msg->id;

    auto it = target_map_.find(id);
    if (it == target_map_.end()) {
      // 새로운 ID 등록
      TARGET_INFO info;
      info.id = id;
      info.fertilized = false;
      info.x = msg->pose.pose.position.x;
      info.y = msg->pose.pose.position.y;
      info.z = msg->pose.pose.position.z;
      info.last_updated = node_->now(); 

      target_map_[id] = info;
      RCLCPP_INFO(node_->get_logger(), "New target ID %d added.", id);
    } else {
    // 기존 객체 처리
      TARGET_INFO &current = it->second;

      if (current.fertilized) return;

      float dx = current.x - msg->pose.pose.position.x;
      float dy = current.y - msg->pose.pose.position.y;
      float dz = current.z - msg->pose.pose.position.z;
      
      float dist_sq = dx * dx + dy * dy + dz * dz;

      if (dist_sq < POSITION_EPSILON * POSITION_EPSILON) return;

      current.x = msg->pose.pose.position.x;
      current.y = msg->pose.pose.position.y;
      current.z = msg->pose.pose.position.z;
      current.last_updated = node_->now(); 
      
      RCLCPP_INFO(node_->get_logger(), "Updated position of ID %d.", id);
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_joint_state_ = *msg;
  }
  
  void check_target(){

     if (target_map_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "⚠️ 대상 객체가 없습니다. check_target()을 종료합니다.");
      return;
    }
    for (auto& [id, obj] : target_map_) {
    if(obj.fertilized) continue;
    else{
        obj.z -= 0.08f;  // Z값 감소
      RCLCPP_INFO(node_->get_logger(), "ID %d의 Z값 감소: %.3f", id, obj.z);
      bumblebee_interfaces::msg::LiftControl lift_msg;
      lift_msg.flag = 1;
      lift_msg.value = 105.0f;
      lift_pub_->publish(lift_msg);
      std::this_thread::sleep_for(std::chrono::seconds(5));
      printTargetMap();
      manipulation();
        lift_msg.flag = 1;
                  lift_msg.value = 104.0f;
                  lift_pub_ -> publish(lift_msg);
                  std::this_thread::sleep_for(std::chrono::seconds(5));
      break;
    }
    }

  }
/*
  void check_target(){

    if (target_map_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "⚠️ 대상 객체가 없습니다. check_target()을 종료합니다.");
      return;
    }
    bool found_target = false;

    for (auto& [id, obj] : target_map_) {
        if (obj.fertilized) continue;

        else{
        obj.z -= 0.08f;  // Z값 보정
        RCLCPP_INFO(node_->get_logger(), "ID %d의 Z값 감소: %.3f", id, obj.z);

        // 🔼 리프트 상승
        bumblebee_interfaces::msg::LiftControl lift_msg;
        lift_msg.flag = 1;
        lift_msg.value = 105.0f;
        lift_pub_->publish(lift_msg);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 🎯 디버깅 출력 및 매니퓰레이터 동작
        printTargetMap();
        manipulation();

        found_target = true;
        
      }
    }
    // ✅ 모든 객체가 수분 완료되었는지 확인
    if (!found_target) {
        bool all_done = true;
        for (const auto& [id, obj] : target_map_) {
            if (!obj.fertilized) {
                all_done = false;
                break;
            }
        }

        if (all_done) {
            RCLCPP_INFO(node_->get_logger(), "🎉 모든 객체 수분 완료. 리프트를 하강시킵니다.");

            bumblebee_interfaces::msg::LiftControl lift_msg;
            lift_msg.flag = 1;
            lift_msg.value = 104.0f;  // ⬇ 최하단 값으로 하강 (필요 시 수정)
            lift_pub_->publish(lift_msg);
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}
*/
  void manipulation(){

    if(left_vel != 0.0 || right_vel != 0.0)
      return;
    
    RCLCPP_WARN(node_->get_logger(), "실행 발생!");

  for (auto& [id, obj] : target_map_) {
    if (obj.fertilized || (obj.z >1.05)) continue;

    if (last_joint_state_.name.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Joint state not received. Skipping ID %d", id);
      continue;
    }

    std_msgs::msg::Bool busy_msg;
    busy_msg.data = true;
    busy_pub_->publish(busy_msg);


    // IK, planning, execution ... (이하 기존 로직 동일)


    // ✅ RobotState 생성 및 IK 기반 자세 계산
    moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(kinematic_model_));
    current_state->setToDefaultValues();

    const moveit::core::JointModelGroup* joint_model_group =
        kinematic_model_->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::msg::Pose pose;
    pose.position.x = obj.x - 0.015; // 오프셋 값
    pose.position.y = obj.y;
    pose.position.z = obj.z + 0.082; // 오프셋 값
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    std::vector<double> solution;
    bool found_ik = false;

    if (current_state->setFromIK(joint_model_group, pose, move_group_.getEndEffectorLink(), 0.1)) {
      current_state->copyJointGroupPositions(joint_model_group, solution);
      found_ik = true;
    }

    if (!found_ik) {
      RCLCPP_WARN(node_->get_logger(), "❌ IK 해를 찾을 수 없음. ID %d", id);
      continue;
    }

    // ✅ 플래닝 및 실행
    move_group_.setJointValueTarget(solution);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      if (move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        obj.fertilized = true;
        RCLCPP_INFO(node_->get_logger(), "✅ 수분 위치 도달 ID %d", id);
      } else {
        RCLCPP_WARN(node_->get_logger(), "❌ 실행 실패 ID %d", id);
        continue;
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "❌ 계획 실패 ID %d", id);
      continue;
    }

    move_group_.clearPoseTargets();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ✅ 수분 동작 수행
    perform_pollination_with_joint7(solution, 0.177266);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    perform_pollination_with_joint7(solution, -0.177266);
     std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ✅ 초기 자세 복귀
    search_JointState();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    busy_msg.data = false;
    busy_pub_->publish(busy_msg);
  }
  check_target();
  }

  void processUnfertilizedTargets() {
    
    manipulation();
}



  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;

  rclcpp::Subscription<bumblebee_interfaces::msg::ManipulatorTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr auto_exec_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr busy_pub_;
  rclcpp::Publisher<bumblebee_interfaces::msg::LiftControl>::SharedPtr lift_pub_;

  sensor_msgs::msg::JointState last_joint_state_;
  std::map<int, TARGET_INFO> target_map_;
  double left_vel = 0.0;
  double right_vel = 0.0;
  int lift_flag;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_auto_executor_node");
  MoveGroupInterfaceNode move_group_interface_node(node);
  //move_group_interface_node.refill_flower();
  //std::this_thread::sleep_for(std::chrono::seconds(10));
  move_group_interface_node.search_JointState1();
  rclcpp::spin(node);
  rclcpp::shutdown();

  move_group_interface_node.printTargetMap();
    
   RCLCPP_INFO(node->get_logger(), "💡 프로그램 종료. 최종 Joint 값 출력:");
   const auto& joint_state = move_group_interface_node.getLastJointState();
   for (size_t i = 0; i < joint_state.name.size(); ++i) {
     RCLCPP_INFO(node->get_logger(), "🦿 %s = %.8f", joint_state.name[i].c_str(), joint_state.position[i]);
   }
  return 0;
}
