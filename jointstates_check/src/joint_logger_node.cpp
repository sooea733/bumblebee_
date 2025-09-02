#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>

class JointStateLogger : public rclcpp::Node
{
public:
  JointStateLogger()
  : Node("joint_state_logger")
  {
    // 모니터링할 조인트 이름 설정
    target_joint_right_ = "wheel_right_joint";
    target_joint_left_ = "wheel_left_joint";

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointStateLogger::joint_state_callback, this, std::placeholders::_1));
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool right_zero = false;
    bool left_zero = false;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& joint_name = msg->name[i];
      double velocity = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;

      if (joint_name == target_joint_right_ && velocity == 0.0) {
        right_zero = true;
      }

      if (joint_name == target_joint_left_ && velocity == 0.0) {
        left_zero = true;
      }
    }

    if (right_zero && left_zero) {
      test();
    }
  }

  void test()
  {
    RCLCPP_INFO(this->get_logger(), "ㅎㅇ");
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::string target_joint_right_;
  std::string target_joint_left_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateLogger>());
  rclcpp::shutdown();
  return 0;
}
